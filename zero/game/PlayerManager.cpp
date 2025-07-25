#include "PlayerManager.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <zero/game/Buffer.h>
#include <zero/game/ChatController.h>
#include <zero/game/Clock.h>
#include <zero/game/GameEvent.h>
#include <zero/game/InputState.h>
#include <zero/game/KDTree.h>
#include <zero/game/Logger.h>
#include <zero/game/Radar.h>
#include <zero/game/ShipController.h>
#include <zero/game/Soccer.h>
#include <zero/game/WeaponManager.h>
#include <zero/game/net/Connection.h>
#include <zero/game/net/PacketDispatcher.h>
#include <zero/game/net/security/Checksum.h>

namespace zero {

constexpr float kAnimDurationShipWarp = 0.5f;
constexpr float kAnimDurationShipExplode = 0.8f;
constexpr float kAnimDurationBombFlash = 0.12f;

static void OnPlayerIdPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnPlayerIdChange(pkt, size);
}

static void OnPlayerEnterPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnPlayerEnter(pkt, size);
}

static void OnPlayerLeavePkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnPlayerLeave(pkt, size);
}

static void OnJoinGamePkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->received_initial_list = true;
}

static void OnPlayerFreqAndShipChangePkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnPlayerFreqAndShipChange(pkt, size);
}

static void OnPlayerFrequencyChangePkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnPlayerFrequencyChange(pkt, size);
}

static void OnLargePositionPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnLargePositionPacket(pkt, size);
}

static void OnSmallPositionPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnSmallPositionPacket(pkt, size);
}

static void OnBatchedSmallPositionPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnBatchedSmallPositionPacket(pkt, size);
}

static void OnBatchedLargePositionPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnBatchedLargePositionPacket(pkt, size);
}

static void OnPlayerDeathPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnPlayerDeath(pkt, size);
}

static void OnFlagDropPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnFlagDrop(pkt, size);
}

static void OnCreateTurretLinkPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnCreateTurretLink(pkt, size);
}

static void OnDestroyTurretLinkPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;
  manager->OnDestroyTurretLink(pkt, size);
}

static void UnstuckSelf(PlayerManager& pm, Player& self) {
  if (self.ship < 8) {
    float radius = pm.connection.settings.ShipSettings[self.ship].GetRadius();

    // Move us out of the wall if the new position is inside.
    while (pm.connection.map.IsColliding(self.position, radius, self.frequency)) {
      self.position = Vector2f(floorf(self.position.x - 1), floorf(self.position.y - 1));

      if (self.position.x < 0) {
        self.position.x = 0;
        break;
      }

      if (self.position.y < 0) {
        self.position.y = 0;
        break;
      }
    }
  }
}
static void OnSetCoordinatesPkt(void* user, u8* pkt, size_t size) {
  PlayerManager* manager = (PlayerManager*)user;

  Player* self = manager->GetSelf();

  if (!self || size < 5) return;

  u16 x = *(u16*)(pkt + 1);
  u16 y = *(u16*)(pkt + 3);

  self->position.x = (float)x + 0.5f;
  self->position.y = (float)y + 0.5f;
  self->velocity.x = 0.0f;
  self->velocity.y = 0.0f;
  self->togglables |= Status_Flash;
  self->warp_anim_t = 0.0f;

  UnstuckSelf(*manager, *self);
  Event::Dispatch(TeleportEvent(*self));

  if (manager->connection.map.GetTileId(self->position) == kTileIdSafe) {
    if (!(self->togglables & Status_Safety)) {
      Event::Dispatch(SafeEnterEvent(self->position));
    }

    self->togglables |= Status_Safety;
  } else {
    if (self->togglables & Status_Safety) {
      Event::Dispatch(SafeLeaveEvent(self->position));
    }

    self->togglables &= ~Status_Safety;
  }

  manager->SendPositionPacket();
}

inline bool IsPlayerVisible(Player& self, u32 self_freq, Player& player) {
  if (self_freq == player.frequency) return true;

  return (!(player.togglables & Status_Cloak)) || (self.togglables & Status_XRadar);
}

PlayerManager::PlayerManager(MemoryArena& perm_arena, Connection& connection, PacketDispatcher& dispatcher)
    : perm_arena(perm_arena), connection(connection) {
  dispatcher.Register(ProtocolS2C::PlayerId, OnPlayerIdPkt, this);
  dispatcher.Register(ProtocolS2C::PlayerEntering, OnPlayerEnterPkt, this);
  dispatcher.Register(ProtocolS2C::PlayerLeaving, OnPlayerLeavePkt, this);
  dispatcher.Register(ProtocolS2C::JoinGame, OnJoinGamePkt, this);
  dispatcher.Register(ProtocolS2C::TeamAndShipChange, OnPlayerFreqAndShipChangePkt, this);
  dispatcher.Register(ProtocolS2C::FrequencyChange, OnPlayerFrequencyChangePkt, this);
  dispatcher.Register(ProtocolS2C::LargePosition, OnLargePositionPkt, this);
  dispatcher.Register(ProtocolS2C::SmallPosition, OnSmallPositionPkt, this);
  dispatcher.Register(ProtocolS2C::BatchedSmallPosition, OnBatchedSmallPositionPkt, this);
  dispatcher.Register(ProtocolS2C::BatchedLargePosition, OnBatchedLargePositionPkt, this);
  dispatcher.Register(ProtocolS2C::PlayerDeath, OnPlayerDeathPkt, this);
  dispatcher.Register(ProtocolS2C::DropFlag, OnFlagDropPkt, this);
  dispatcher.Register(ProtocolS2C::SetCoordinates, OnSetCoordinatesPkt, this);
  dispatcher.Register(ProtocolS2C::CreateTurret, OnCreateTurretLinkPkt, this);
  dispatcher.Register(ProtocolS2C::DestroyTurret, OnDestroyTurretLinkPkt, this);

  memset(player_lookup, 0xFF, sizeof(player_lookup));
}

void PlayerManager::Update(float dt) {
  zero::Tick current_tick = GetCurrentTick();
  Player* self = GetPlayerById(player_id);

  if (!self) return;

  for (size_t i = 0; i < this->player_count; ++i) {
    Player* player = this->players + i;

    if (player->ship >= 8) continue;

    SimulatePlayer(*player, dt, false);

    player->explode_anim_t += dt;
    player->warp_anim_t += dt;
    player->bombflash_anim_t += dt;

    if (player->enter_delay > 0.0f) {
      player->enter_delay -= dt;

      if (player->explode_anim_t >= kAnimDurationShipExplode) {
        if (player != self) {
          player->position = Vector2f(0, 0);
          player->lerp_time = 0.0f;
        }

        player->velocity = Vector2f(0, 0);
      }

      if (player == self && player->enter_delay <= 0.0f) {
        if (connection.settings.EnterDelay > 0) {
          Spawn();
          player->warp_anim_t = 0.0f;
        } else {
          player->energy = 1;
        }
      }
    }
  }

  s32 position_delay = 100;

  if (self && self->ship != 8) {
    position_delay = connection.settings.SendPositionDelay;

    if (position_delay < 5) {
      position_delay = 5;
    }

    if (self->enter_delay > 0.0f) {
      position_delay = 50;
    }
  }

  s32 server_timestamp = MAKE_TICK(current_tick + connection.time_diff);
  if (connection.login_state == Connection::LoginState::Complete && connection.joined_arena &&
      abs(server_timestamp - last_position_tick) >= position_delay) {
    SendPositionPacket();
  }

  if (damage_count > 0 && TICK_DIFF(current_tick, last_send_damage_tick) >= 10) {
    connection.SendDamage(damage_count, damages);
    damage_count = 0;
    last_send_damage_tick = current_tick;
  }
}

void PlayerManager::Render(Camera& camera, SpriteRenderer& renderer) {
  Player* self = GetPlayerById(player_id);

  if (!self) return;

  u32 self_freq = self->frequency;

  // Draw player ships
  for (size_t i = 0; i < this->player_count; ++i) {
    Player* player = this->players + i;

    if (player->ship == 8) continue;
    if (player->position == Vector2f(0, 0)) continue;
    if (player->attach_parent != kInvalidPlayerId) continue;

    if (explode_animation.IsAnimating(player->explode_anim_t)) {
      SpriteRenderable& renderable = explode_animation.GetFrame(player->explode_anim_t);
      Vector2f position = player->position - renderable.dimensions * (0.5f / 16.0f);

      renderer.Draw(camera, renderable, position, Layer::AfterShips);
    } else if (player->enter_delay <= 0.0f) {
      if (IsSynchronized(*player) && IsPlayerVisible(*self, self_freq, *player)) {
        size_t index = player->ship * 40 + (u8)(player->orientation * 40.0f);

        Vector2f offset = Graphics::ship_sprites[index].dimensions * (0.5f / 16.0f);
        Vector2f position = player->position.PixelRounded() - offset.PixelRounded();

        renderer.Draw(camera, Graphics::ship_sprites[index], position, Layer::Ships);
      }

      AttachInfo* info = player->children;

      while (info) {
        Player* child = GetPlayerById(info->player_id);

        if (child && IsSynchronized(*child) && IsPlayerVisible(*self, self_freq, *child)) {
          size_t index = (size_t)(child->orientation * 40.0f);

          Vector2f offset = Graphics::turret_sprites[index].dimensions * (0.5f / 16.0f);
          Vector2f position = player->position.PixelRounded() - offset.PixelRounded();

          renderer.Draw(camera, Graphics::turret_sprites[index], position, Layer::Ships);
        }

        info = info->next;
      }

      if (warp_animation.IsAnimating(player->warp_anim_t)) {
        SpriteRenderable& renderable = warp_animation.GetFrame(player->warp_anim_t);
        Vector2f position = player->position - renderable.dimensions * (0.5f / 16.0f);

        renderer.Draw(camera, renderable, position, Layer::AfterShips);
      }

      if (bombflash_animation.IsAnimating(player->bombflash_anim_t)) {
        SpriteRenderable& renderable = bombflash_animation.GetFrame(player->bombflash_anim_t);
        Vector2f heading = OrientationToHeading((u8)(player->orientation * 40.0f));
        ShipSettings& ship_settings = connection.settings.ShipSettings[player->ship];

        Vector2f position =
            player->position + heading * ship_settings.GetRadius() - renderable.dimensions * (0.5f / 16.0f);

        renderer.Draw(camera, renderable, position, Layer::Weapons);
      }
    } else if (player == self && player->enter_delay > 0 && !explode_animation.IsAnimating(player->explode_anim_t)) {
      char output[256];
      sprintf(output, "%.1f", player->enter_delay);
      renderer.PushText(camera, output, TextColor::DarkRed, camera.position, Layer::TopMost, TextAlignment::Center);
    }
  }

  // Draw player names - This is done in separate loop to batch sprite sheet renderables
  for (size_t i = 0; i < this->player_count; ++i) {
    Player* player = this->players + i;

    if (player->ship == 8) continue;
    if (player->position == Vector2f(0, 0)) continue;
    if (player->attach_parent != kInvalidPlayerId) continue;

    Vector2f position = player->position;

    // Don't render the player's name if they aren't synchronized, but still render their children
    if (IsSynchronized(*player)) {
      RenderPlayerName(camera, renderer, *self, *player, position, false);

      float max_energy = (float)ship_controller->ship.energy;
      if (player->id == player_id && player->energy < max_energy * 0.5f) {
        position += Vector2f(0, 12.0f / 16.0f);
      }
    }

    AttachInfo* info = player->children;

    while (info) {
      position += Vector2f(0, 12.0f / 16.0f);

      Player* child = GetPlayerById(info->player_id);

      if (child && IsSynchronized(*child)) {
        RenderPlayerName(camera, renderer, *self, *child, position, false);
      }

      info = info->next;
    }
  }
}

void PlayerManager::RenderPlayerName(Camera& camera, SpriteRenderer& renderer, Player& self, Player& player,
                                     const Vector2f& position, bool is_decoy) {
  if (player.ship == 8) return;
  if (player.position == Vector2f(0, 0)) return;

  u32 tick = GetCurrentTick();
  u32 self_freq = self.frequency;

  if (!IsPlayerVisible(self, self_freq, player)) return;

  if (player.enter_delay <= 0.0f) {
    size_t render_ship = player.ship;

    if (player.attach_parent != kInvalidPlayerId) {
      Player* parent = GetPlayerById(player.attach_parent);

      if (parent && parent->ship != 8) {
        render_ship = parent->ship;
      }
    }

    size_t index = render_ship * 40 + (u8)(player.orientation * 40.0f);
    Vector2f offset = Graphics::ship_sprites[index].dimensions * (0.5f / 16.0f);

    offset = offset.PixelRounded();

    char display[80];

    bool display_ball = player.ball_carrier && !is_decoy;

    if (player.flags > 0) {
      sprintf(display, "%s(%d:%d)[%d] %s", player.name, player.bounty, player.flags, player.ping * 10,
              display_ball ? "(Ball)" : "");
    } else {
      sprintf(display, "%s(%d)[%d] %s", player.name, player.bounty, player.ping * 10, display_ball ? "(Ball)" : "");
    }

    TextColor color = TextColor::Blue;

    if (player.frequency == self_freq) {
      color = TextColor::Yellow;
    } else if (player.flags > 0 || (player.ball_carrier && !is_decoy)) {
      color = TextColor::DarkRed;
    }

    Vector2f current_position = position.PixelRounded() + offset;

    if (!is_decoy) {
      if (player.ball_carrier && player.id == player_id &&
          connection.settings.ShipSettings[player.ship].SoccerBallThrowTimer > 0) {
        char ball_time_output[16];

        sprintf(ball_time_output, "%.1f", soccer->carry_timer);

        renderer.PushText(camera, ball_time_output, TextColor::Red, current_position, Layer::Ships);
        current_position.y += (12.0f / 16.0f);
      }

      float max_energy = (float)ship_controller->ship.energy;

      if (player.id == player_id && player.energy < max_energy * 0.5f) {
        TextColor energy_color = player.energy < max_energy * 0.25f ? TextColor::DarkRed : TextColor::Yellow;
        char energy_output[16];
        sprintf(energy_output, "%d", (u32)player.energy);

        renderer.PushText(camera, energy_output, energy_color, current_position, Layer::Ships);

        current_position.y += (12.0f / 16.0f);
      } else if (player.id != player_id && TICK_DIFF(tick, player.last_extra_timestamp) < kExtraDataTimeout) {
        char energy_output[16];
        sprintf(energy_output, "%d", (u32)player.energy);
        Vector2f energy_p = position.PixelRounded() + Vector2f(-0.5f, offset.y);

        float initial_energy = (float)connection.settings.ShipSettings[player.ship].InitialEnergy;
        TextColor color = TextColor::Blue;

        if (player.energy < initial_energy / 4.0f) {
          color = TextColor::DarkRed;
        } else if (player.energy < initial_energy / 2.0f) {
          color = TextColor::Yellow;
        }

        renderer.PushText(camera, energy_output, color, energy_p, Layer::Ships, TextAlignment::Right);
      }
    }

    renderer.PushText(camera, display, color, current_position.PixelRounded(), Layer::Ships);
  }
}

void PlayerManager::PushDamage(PlayerId shooter_id, WeaponData weapon_data, int energy, int damage) {
  if (damage_count >= ZERO_ARRAY_SIZE(damages)) return;

  Damage& dmg = damages[damage_count++];

  dmg.timestamp = connection.GetServerTick();
  dmg.shooter_id = shooter_id;
  dmg.weapon_data = weapon_data;
  dmg.energy = (s16)energy;
  dmg.damage = (s16)damage;
}

void PlayerManager::SendPositionPacket() {
  u8 data[kMaxPacketSize];
  NetworkBuffer buffer(data, kMaxPacketSize);

  Player* player = GetPlayerById(player_id);

  assert(player);

  u16 x = (u16)(player->position.x * 16.0f);
  u16 y = (u16)(player->position.y * 16.0f);
  u16 vel_x = (u16)(player->velocity.x * 16.0f * 10.0f);
  u16 vel_y = (u16)(player->velocity.y * 16.0f * 10.0f);
  u16 weapon = *(u16*)&player->weapon;
  u16 energy = (u16)player->energy;
  u8 direction = (u8)(player->orientation * 40.0f);
  u16 bounty = player->bounty;
  u8 togglables = player->togglables;

  if (player->ship != 8 && player->enter_delay > 0.0f) {
    x = 0xFFFF;
    y = 0xFFFF;
    vel_x = 0;
    vel_y = 0;
    direction = 0;
    togglables = 0x80;
    energy = 0;
    bounty = 0;
    weapon = 0;
  }

  u32 local_timestamp = GetCurrentTick();

  s32 server_timestamp = MAKE_TICK(local_timestamp + connection.time_diff);

  if (player->attach_parent != kInvalidPlayerId) {
    vel_x = 0;
    vel_y = 0;

    Player* parent = GetPlayerById(player->attach_parent);

    if (parent) {
      // We can't send more position packets to server while waiting for the attach request to go through.
      if (!IsSynchronized(*parent)) {
        last_position_tick = server_timestamp;
        return;
      }

      // If we are requesting an attach and we got our parent's position, drop our energy for the attach operation.
      if (requesting_attach) {
        player->energy = player->energy * 0.333f;
        requesting_attach = false;
        Event::Dispatch(PlayerAttachEvent(*player, *parent));
      }

      vel_x = (u16)(parent->velocity.x * 16.0f * 10.0f);
      vel_y = (u16)(parent->velocity.y * 16.0f * 10.0f);
    } else {
      player->attach_parent = kInvalidPlayerId;
      requesting_attach = false;
    }
  }

  // Override the timestamp if the time_diff changes or it's being sent on the same tick as last packet.
  // This is necessary because packets will be thrown away server side if the timestamp isn't newer.
  if (server_timestamp <= last_position_tick) {
    server_timestamp = MAKE_TICK(last_position_tick + 1);
  }

  buffer.WriteU8(0x03);               // Type
  buffer.WriteU8(direction);          // Direction
  buffer.WriteU32(server_timestamp);  // Timestamp
  buffer.WriteU16(vel_x);             // X velocity
  buffer.WriteU16(y);                 // Y
  buffer.WriteU8(0);                  // Checksum
  buffer.WriteU8(togglables);         // Togglables
  buffer.WriteU16(x);                 // X
  buffer.WriteU16(vel_y);             // Y velocity
  buffer.WriteU16(bounty);            // Bounty
  buffer.WriteU16(energy);            // Energy
  buffer.WriteU16(weapon);            // Weapon info

  u8 checksum = WeaponChecksum(buffer.data, buffer.GetSize());
  buffer.data[10] = checksum;

  if (connection.extra_position_info || connection.settings.ExtraPositionData) {
    buffer.WriteU16(energy);
    buffer.WriteU16(connection.ping / 10);
    buffer.WriteU16(player->flag_timer / 100);

    struct {
      u32 shields : 1;
      u32 super : 1;
      u32 bursts : 4;
      u32 repels : 4;
      u32 thors : 4;
      u32 bricks : 4;
      u32 decoys : 4;
      u32 rockets : 4;
      u32 portals : 4;
      u32 padding : 2;
    } item_info = {};

    item_info.bursts = ship_controller->ship.bursts;
    item_info.repels = ship_controller->ship.repels;
    item_info.thors = ship_controller->ship.thors;
    item_info.bricks = ship_controller->ship.bricks;
    item_info.decoys = ship_controller->ship.decoys;
    item_info.rockets = ship_controller->ship.rockets;
    item_info.portals = ship_controller->ship.portals;

    buffer.WriteU32(*(u32*)&item_info);
  }

  connection.Send(buffer);
  last_position_tick = server_timestamp;
  player->togglables &= ~Status_Flash;
}

Player* PlayerManager::GetSelf() {
  return GetPlayerById(player_id);
}

Player* PlayerManager::GetPlayerById(u16 id, size_t* index) {
  u16 player_index = player_lookup[id];

  if (player_index < kInvalidPlayerId) {
    if (index) {
      *index = player_index;
    }

    return players + player_index;
  }

  return nullptr;
}

Player* PlayerManager::GetPlayerByName(const char* name) {
  for (size_t i = 0; i < player_count; ++i) {
    Player* player = players + i;

    if (strcmp(player->name, name) == 0) {
      return player;
    }
  }

  return nullptr;
}

void PlayerManager::OnPlayerIdChange(u8* pkt, size_t size) {
  player_id = *(u16*)(pkt + 1);
  Log(LogLevel::Debug, "Player id: %d", player_id);

  this->player_count = 0;
  this->received_initial_list = false;
  this->kdtree = nullptr;

  memset(player_lookup, 0xFF, sizeof(player_lookup));
}

void PlayerManager::OnPlayerEnter(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u8 ship = buffer.ReadU8();
  u8 audio = buffer.ReadU8();
  char* name = buffer.ReadString(20);
  char* squad = buffer.ReadString(20);

  Player* existing_player = GetPlayerByName(name);
  if (existing_player) {
    // This can happen on servers that mistakenly send the enter packet after already including them in the initial
    // list.
    RemovePlayer(existing_player);
  }

  size_t player_index = player_count++;

  assert(player_index < ZERO_ARRAY_SIZE(players));

  Player* player = players + player_index;

  memset(player, 0, sizeof(Player));

  player->ship = ship;

  memcpy(player->name, name, 20);
  player->name[20] = 0;
  memcpy(player->squad, squad, 20);
  player->squad[20] = 0;

  player->kill_points = buffer.ReadU32();
  player->flag_points = buffer.ReadU32();
  player->id = buffer.ReadU16();
  player->frequency = buffer.ReadU16();
  player->wins = buffer.ReadU16();
  player->losses = buffer.ReadU16();
  player->attach_parent = buffer.ReadU16();
  player->flags = buffer.ReadU16();
  player->koth = buffer.ReadU8();
  player->timestamp = kInvalidSmallTick;

  player->warp_anim_t = kAnimDurationShipWarp;
  player->explode_anim_t = kAnimDurationShipExplode;
  player->bombflash_anim_t = kAnimDurationBombFlash;

  player_lookup[player->id] = (u16)player_index;

  Log(LogLevel::Info, "%s [%d] entered arena", name, player->id);

  if (player->attach_parent != kInvalidPlayerId) {
    Player* destination = GetPlayerById(player->attach_parent);

    if (destination) {
      AttachPlayer(*player, *destination);
    }
  }

  if (chat_controller && received_initial_list) {
    chat_controller->AddMessage(ChatType::Arena, "%s entered arena", player->name);
  }

  Event::Dispatch(PlayerEnterEvent(*player));
}

void PlayerManager::OnPlayerLeave(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u16 pid = buffer.ReadU16();
  Player* player = GetPlayerById(pid);

  RemovePlayer(player);
}

void PlayerManager::RemovePlayer(Player* player) {
  if (!player) return;

  size_t index = (size_t)(player - players);

  weapon_manager->ClearWeapons(*player);

  Log(LogLevel::Info, "%s left arena", player->name);

  DetachPlayer(*player);
  DetachAllChildren(*player);

  if (chat_controller) {
    chat_controller->AddMessage(ChatType::Arena, "%s left arena", player->name);
  }

  Event::Dispatch(PlayerLeaveEvent(*player));

  // Swap the last player in the list's lookup to point to their new index
  assert(index < 1024);

  player_lookup[players[player_count - 1].id] = (u16)index;
  player_lookup[player->id] = kInvalidPlayerId;

  players[index] = players[--player_count];
}

void PlayerManager::OnPlayerDeath(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u8 green_id = buffer.ReadU8();
  u16 killer_id = buffer.ReadU16();
  u16 killed_id = buffer.ReadU16();
  u16 bounty = buffer.ReadU16();
  u16 flag_transfer = buffer.ReadU16();

  Player* killed = GetPlayerById(killed_id);
  Player* killer = GetPlayerById(killer_id);

  if (killed) {
    // Hide the player until they send a new position packet
    killed->enter_delay = (connection.settings.EnterDelay / 100.0f) + kAnimDurationShipExplode;
    killed->explode_anim_t = 0.0f;
    killed->flags = 0;
    killed->flag_timer = 0;
    killed->ball_carrier = false;
    killed->energy = 0;

    DetachPlayer(*killed);
    DetachAllChildren(*killed);
  }

  if (killer && killer != killed) {
    killer->flags += flag_transfer;

    if (flag_transfer > 0) {
      killer->flag_timer = connection.settings.FlagDropDelay;
    }

    if (killer->id == player_id && killed && killed->bounty > 0) {
      killer->bounty += connection.settings.BountyIncreaseForKill;
    }
  }

  if (killer && killed) {
    Event::Dispatch(PlayerDeathEvent(*killed, *killer, bounty, flag_transfer));
  }
}

static inline u32 HashName(Player& player) {
  u32 hash = 0;

  const char* c = player.name;

  for (; *c; ++c) {
    hash += *c;
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);

  return hash;
}

void PlayerManager::Spawn(bool reset) {
  Player* self = GetSelf();

  if (!self) return;

  u8 ship = self->ship;
  u32 spawn_count = 0;

  for (size_t i = 0; i < ZERO_ARRAY_SIZE(connection.settings.SpawnSettings); ++i) {
    if (connection.settings.SpawnSettings[i].X != 0 || connection.settings.SpawnSettings[i].Y != 0 ||
        connection.settings.SpawnSettings[i].Radius != 0) {
      ++spawn_count;
    }
  }

  float ship_radius = connection.settings.ShipSettings[ship].GetRadius();

  // Create a hash based on our name so we can offset the random seed.
  // This is to stop many bots ran at the same time from generating the same positions.
  u32 hash = HashName(*self);
  u32 rand_seed = rand() + hash;

  if (spawn_count == 0) {
    // Default position to center of map if no location could be found.
    self->position = Vector2f(512, 512);

    for (size_t i = 0; i < 100; ++i) {
      u16 x = 0;
      u16 y = 0;

      switch (connection.settings.RadarMode) {
        case 1:
        case 3: {
          VieRNG rng = {(s32)rand_seed};
          u8 rng_x = (u8)rng.GetNext();
          u8 rng_y = (u8)rng.GetNext();

          x = (self->frequency & 1) * 0x300 + rng_x;
          y = rng_y + 0x100;
        } break;
        case 2:
        case 4: {
          VieRNG rng = {(s32)rand_seed};
          u8 rng_x = (u8)rng.GetNext();
          u8 rng_y = (u8)rng.GetNext();

          x = (self->frequency & 1) * 0x300 + rng_x;
          y = ((self->frequency / 2) & 1) * 0x300 + rng_y;
        } break;
        default: {
          u32 spawn_radius = (((u32)player_count / 8) * 0x2000 + 0x400) / 0x60 + 0x100;

          if (spawn_radius > (u32)connection.settings.WarpRadiusLimit) {
            spawn_radius = (u32)connection.settings.WarpRadiusLimit;
          }

          if (spawn_radius < 3) {
            spawn_radius = 3;
          }

          VieRNG rng = {(s32)rand_seed};
          x = rng.GetNext() % (spawn_radius - 2) - 9 + ((0x400 - spawn_radius) / 2) + (rand() % 0x14);
          y = rng.GetNext() % (spawn_radius - 2) - 9 + ((0x400 - spawn_radius) / 2) + (rand() % 0x14);
        } break;
      }

      Vector2f spawn((float)x, (float)y);

      if (connection.map.CanFit(spawn, ship_radius, self->frequency)) {
        self->position = spawn;
        break;
      }
    }
  } else {
    u32 spawn_index = self->frequency % spawn_count;

    float x_center = (float)connection.settings.SpawnSettings[spawn_index].X;
    float y_center = (float)connection.settings.SpawnSettings[spawn_index].Y;
    int radius = connection.settings.SpawnSettings[spawn_index].Radius;

    if (x_center == 0) {
      x_center = 512;
    } else if (x_center < 0) {
      x_center += 1024;
    }
    if (y_center == 0) {
      y_center = 512;
    } else if (y_center < 0) {
      y_center += 1024;
    }

    // Default to exact center in the case that a random position wasn't found
    self->position = Vector2f(x_center, y_center);

    if (radius > 0) {
      // Try 100 times to spawn in a random spot.
      for (int i = 0; i < 100; ++i) {
        u32 xrand = ((u32)rand() + hash);
        u32 yrand = ((u32)rand() + hash);

        float x_offset = (float)((int)(xrand % (radius * 2)) - radius);
        float y_offset = (float)((int)(yrand % (radius * 2)) - radius);

        Vector2f spawn(x_center + x_offset, y_center + y_offset);

        if (connection.map.CanFit(spawn, ship_radius, self->frequency)) {
          self->position = spawn;
          break;
        }
      }
    }
  }

  if (reset) {
    ship_controller->ResetShip();
  }

  self->togglables |= Status_Flash;
  self->warp_anim_t = 0.0f;
  self->velocity = Vector2f(0, 0);

  Event::Dispatch(SpawnEvent(*self));
}

void PlayerManager::OnPlayerFrequencyChange(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u16 pid = buffer.ReadU16();
  u16 frequency = buffer.ReadU16();

  Player* player = GetPlayerById(pid);

  if (player) {
    DetachPlayer(*player);
    DetachAllChildren(*player);

    u16 old_freq = player->frequency;

    player->frequency = frequency;
    player->velocity = Vector2f(0, 0);

    player->lerp_time = 0.0f;
    player->warp_anim_t = 0.0f;
    player->enter_delay = 0.0f;
    player->flags = 0;
    player->ball_carrier = false;
    player->energy = 0;

    weapon_manager->ClearWeapons(*player);

    Event::Dispatch(PlayerFreqAndShipChangeEvent(*player, old_freq, frequency, player->ship, player->ship));

    if (player->id == player_id) {
      Spawn(true);
    }
  }
}

void PlayerManager::OnPlayerFreqAndShipChange(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u8 ship = buffer.ReadU8();
  u16 pid = buffer.ReadU16();
  u16 freq = buffer.ReadU16();

  Player* player = GetPlayerById(pid);

  if (player) {
    DetachPlayer(*player);
    DetachAllChildren(*player);

    u16 old_freq = player->frequency;
    u8 old_ship = player->ship;

    player->ship = ship;
    player->frequency = freq;
    player->velocity = Vector2f(0, 0);

    player->lerp_time = 0.0f;
    player->warp_anim_t = 0.0f;
    player->enter_delay = 0.0f;
    player->flags = 0;
    player->ball_carrier = false;
    player->energy = 0;

    weapon_manager->ClearWeapons(*player);

    if (player->id == player_id) {
      Spawn(true);
    }

    // Dispatch this event after spawn so it sends a new position packet with the new ship while pathfinder is building.
    Event::Dispatch(PlayerFreqAndShipChangeEvent(*player, old_freq, freq, old_ship, ship));
  }
}

s32 GetTimestampDiff(Connection& connection, u32 tagged_timestamp) {
  s32 timestamp_diff = TICK_DIFF(connection.GetServerTick(), tagged_timestamp);

  if (timestamp_diff < 0 || timestamp_diff > 4000) {
    timestamp_diff = (connection.ping / 10) / 2;

    if (timestamp_diff > 14) {
      timestamp_diff = 15;
    }
  }

  return timestamp_diff;
}

bool IsNewerPositionPacket(Player* player, u16 timestamp) {
  if (!player) return false;

  if (player->timestamp == kInvalidSmallTick) return true;
  if (SMALL_TICK_GTE(timestamp, player->timestamp)) return true;

  return abs(timestamp - player->timestamp) > 999;
}

void PlayerManager::OnLargePositionPacket(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u8 direction = buffer.ReadU8();
  u16 timestamp = buffer.ReadU16();
  u16 x = buffer.ReadU16();
  s16 vel_y_s16 = (s16)buffer.ReadU16();
  u16 pid = buffer.ReadU16();

  Player* player = GetPlayerById(pid);

  // Put packet timestamp into local time
  u32 server_timestamp = (connection.GetServerTick() & 0x7FFF0000) | timestamp;
  u32 local_timestamp = server_timestamp - connection.time_diff;

  // Throw away bad timestamps so the player doesn't get desynchronized.
  if (TICK_DIFF(local_timestamp, GetCurrentTick()) >= 300) {
    return;
  }

  if (IsNewerPositionPacket(player, timestamp)) {
    player->orientation = direction / 40.0f;
    float vel_y = vel_y_s16 / 16.0f / 10.0f;
    float vel_x = (s16)buffer.ReadU16() / 16.0f / 10.0f;

    Vector2f velocity(vel_x, vel_y);

    u8 checksum = buffer.ReadU8();
    player->togglables = buffer.ReadU8();
    player->ping = buffer.ReadU8();
    u16 y = buffer.ReadU16();
    player->bounty = buffer.ReadU16();

    if (player->togglables & Status_Flash) {
      player->warp_anim_t = 0.0f;
    }

    u16 weapon = buffer.ReadU16();
    memcpy(&player->weapon, &weapon, sizeof(weapon));

    if (weapon != 0) {
      ++connection.weapons_received;
    }

    // Don't force set own energy/latency
    if (player->id != player_id) {
      if (size >= 23) {
        player->last_extra_timestamp = GetCurrentTick();
        player->energy = (float)buffer.ReadU16();
      }

      if (size >= 25) {
        player->s2c_latency = buffer.ReadU16();
      }

      if (size >= 27) {
        player->flag_timer = buffer.ReadU16();
      }

      if (size >= 31) {
        player->items = buffer.ReadU32();
      }
    }

    s32 timestamp_diff = GetTimestampDiff(connection, server_timestamp);

    player->timestamp = timestamp;
    player->ping += timestamp_diff;

    Vector2f pkt_position(x / 16.0f, y / 16.0f);
    OnPositionPacket(*player, pkt_position, velocity, player->ping);
  }
}

void PlayerManager::OnSmallPositionPacket(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();

  u8 direction = buffer.ReadU8();
  u16 timestamp = buffer.ReadU16();
  u16 x = buffer.ReadU16();
  u8 ping = buffer.ReadU8();
  u8 bounty = buffer.ReadU8();
  u16 pid = buffer.ReadU8();

  Player* player = GetPlayerById(pid);

  // Put packet timestamp into local time
  u32 server_timestamp = (connection.GetServerTick() & 0x7FFF0000) | timestamp;
  u32 local_timestamp = server_timestamp - connection.time_diff;

  // Throw away bad timestamps so the player doesn't get desynchronized.
  if (TICK_DIFF(local_timestamp, GetCurrentTick()) >= 300) {
    return;
  }

  // Only perform update if the packet is newer than the previous one.
  if (IsNewerPositionPacket(player, timestamp)) {
    player->orientation = direction / 40.0f;
    player->ping = ping;
    player->bounty = bounty;
    player->togglables = buffer.ReadU8();
    float vel_y = (s16)buffer.ReadU16() / 16.0f / 10.0f;
    u16 y = buffer.ReadU16();
    float vel_x = (s16)buffer.ReadU16() / 16.0f / 10.0f;

    Vector2f velocity(vel_x, vel_y);

    if (player->togglables & Status_Flash) {
      player->warp_anim_t = 0.0f;
    }

    // Don't force set own energy/latency
    if (player->id != player_id) {
      if (size >= 18) {
        player->last_extra_timestamp = GetCurrentTick();
        player->energy = (float)buffer.ReadU16();
      }

      if (size >= 20) {
        player->s2c_latency = buffer.ReadU16();
      }

      if (size >= 22) {
        player->flag_timer = buffer.ReadU16();
      }

      if (size >= 26) {
        player->items = buffer.ReadU32();
      }
    }

    s32 timestamp_diff = GetTimestampDiff(connection, server_timestamp);

    player->timestamp = timestamp;
    player->ping += timestamp_diff;

    Vector2f pkt_position(x / 16.0f, y / 16.0f);
    OnPositionPacket(*player, pkt_position, velocity, player->ping);
  }
}

void PlayerManager::OnBatchedLargePositionPacket(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();  // Type

  u32 server_tick = (GetCurrentTick() + connection.time_diff);

  while (buffer.write - buffer.read >= 11) {
    u16 pid_togglables = buffer.ReadU16();

    u16 player_id = pid_togglables & 0x3FF;
    u8 togglables = (u8)(pid_togglables >> 10);

    u16 packed = buffer.ReadU16();
    u16 direction = (packed >> 10);
    u16 timestamp = (packed & 0x3FF);

    u32 packed_pos = buffer.ReadU32();
    u32 x = packed_pos & 0x3FFF;
    u32 y = (packed_pos >> 0x0E) & 0x3FFF;

    u16 packed_velocity = buffer.ReadU16();
    s16 vel_y = (packed_velocity << 0x12) >> 0x12;

    s8 multiplier = buffer.ReadU8();

    s32 vel_x = ((packed_velocity >> 0x0E) + (multiplier * 4)) * 0x10 + (packed_pos >> 0x1C);

    Vector2f velocity(vel_x / 16.0f / 10.0f, vel_y / 16.0f / 10.0f);
    Vector2f position(x / 16.0f, y / 16.0f);

    // Put packet timestamp into local time
    u32 server_timestamp = (connection.GetServerTick() & 0x7FFFFC00) | timestamp;
    u32 local_timestamp = server_timestamp - connection.time_diff;
    timestamp = server_timestamp & 0xFFFF;

    // Throw away bad timestamps so the player doesn't get desynchronized.
    if (TICK_DIFF(local_timestamp, GetCurrentTick()) >= 300) {
      continue;
    }

    Player* player = GetPlayerById(player_id);

    if (player && IsNewerPositionPacket(player, timestamp)) {
      s32 timestamp_diff = GetTimestampDiff(connection, server_timestamp);

      player->timestamp = timestamp;
      player->orientation = direction / 40.0f;
      // Store the new togglables, but keep the top 2 bits since they aren't sent in this.
      player->togglables = togglables | (player->togglables & 0xC0);

      OnPositionPacket(*player, position, velocity, timestamp_diff);
    }
  }
}

void PlayerManager::OnBatchedSmallPositionPacket(u8* pkt, size_t size) {
  NetworkBuffer buffer(pkt, size, size);

  buffer.ReadU8();  // Type

  u32 server_tick = (GetCurrentTick() + connection.time_diff);

  while (buffer.write - buffer.read >= 10) {
    u8 player_id = buffer.ReadU8();

    u16 packed = buffer.ReadU16();
    u16 direction = (packed >> 10);
    s16 timestamp = (packed & 0x3FF);

    u32 packed_pos = buffer.ReadU32();
    u32 x = packed_pos & 0x3FFF;
    u32 y = (packed_pos >> 0x0E) & 0x3FFF;

    u16 packed_velocity = buffer.ReadU16();
    s16 vel_y = (packed_velocity << 0x12) >> 0x12;

    s8 multiplier = buffer.ReadU8();

    s32 vel_x = ((packed_velocity >> 0x0E) + (multiplier * 4)) * 0x10 + (packed_pos >> 0x1C);

    Vector2f velocity(vel_x / 16.0f / 10.0f, vel_y / 16.0f / 10.0f);
    Vector2f position(x / 16.0f, y / 16.0f);

    // Put packet timestamp into local time
    u32 server_timestamp = (connection.GetServerTick() & 0x7FFFFC00) | timestamp;
    u32 local_timestamp = server_timestamp - connection.time_diff;
    timestamp = server_timestamp & 0xFFFF;

    // Throw away bad timestamps so the player doesn't get desynchronized.
    if (TICK_DIFF(local_timestamp, GetCurrentTick()) >= 300) {
      continue;
    }

    Player* player = GetPlayerById(player_id);

    if (player && IsNewerPositionPacket(player, timestamp)) {
      s32 timestamp_diff = GetTimestampDiff(connection, server_timestamp);

      player->timestamp = timestamp;
      player->orientation = direction / 40.0f;

      OnPositionPacket(*player, position, velocity, timestamp_diff);
    }
  }
}

void PlayerManager::OnPositionPacket(Player& player, const Vector2f& position, const Vector2f& velocity,
                                     s32 sim_ticks) {
  Vector2f previous_pos = player.position;

  // Ignore position packets for self if dead. This exists because Hyperspace transwarp seems to warp the player while
  // dead but doesn't do it in Continuum.
  if (player.id == player_id && player.enter_delay > 0.0f) {
    return;
  }

  // Hard set the new position so we can simulate from it to catch up to where the player would be now after ping ticks
  player.position = position;
  player.velocity = velocity;

  // Clear lerp time so it doesn't affect real simulation.
  player.lerp_time = 0.0f;

  // Client sends ppk to server with server timestamp, server calculates the tick difference on arrival and sets that to
  // ping. The player should be simulated however many ticks it took to reach server plus the tick difference between
  // this client and the server.

  // Simulate per tick because the simulation can be unstable with large dt
  for (int i = 0; i < sim_ticks; ++i) {
    SimulatePlayer(player, (1.0f / 100.0f), true);
  }

  Vector2f projected_pos = player.position;

  // Set the player back to where they were before the simulation so they can be lerped to new position.
  player.position = previous_pos;

  float abs_dx = abs(projected_pos.x - player.position.x);
  float abs_dy = abs(projected_pos.y - player.position.y);

  // Jump to the position if very out of sync or just warped
  if (abs_dx >= 4.0f || abs_dy >= 4.0f || (player.togglables & Status_Flash)) {
    player.position = projected_pos;
    player.lerp_time = 0.0f;

    if (player.togglables & Status_Flash && previous_pos != Vector2f(0, 0)) {
      player.togglables &= ~Status_Flash;
    }
  } else {
    player.lerp_time = 200.0f / 1000.0f;
    player.lerp_velocity = (projected_pos - player.position) * (1.0f / player.lerp_time);
  }

  // We received a packet telling us where we are, so make sure it didn't put is in a wall. (Hyperspace)
  if (player.id == player_id) {
    UnstuckSelf(*this, player);
    Event::Dispatch(TeleportEvent(player));
  }
}

void PlayerManager::OnFlagDrop(u8* pkt, size_t size) {
  u16 player_id = *(u16*)(pkt + 1);

  Player* player = GetPlayerById(player_id);

  if (player) {
    player->flags = 0;
    player->flag_timer = 0;
  }
}

AttachRequestResponse PlayerManager::AttachSelf(Player* destination) {
  if (!destination) return AttachRequestResponse::NoDestination;
  if (soccer->IsCarryingBall()) return AttachRequestResponse::CarryingBall;

  Player* self = GetSelf();

  if (!self) return AttachRequestResponse::UnrecoverableError;

  if (self->attach_parent != kInvalidPlayerId) {
    connection.SendAttachRequest(kInvalidPlayerId);
    DetachPlayer(*self);
    return AttachRequestResponse::DetatchFromParent;
  }

  if (self->children) {
    connection.SendAttachDrop();
    return AttachRequestResponse::DetatchChildren;
  }

  if (self->energy < (float)ship_controller->ship.energy) {
    return AttachRequestResponse::NotEnoughEnergy;
  }

  ShipSettings& src_settings = connection.settings.ShipSettings[self->ship];

  if (self->bounty < src_settings.AttachBounty) {
    return AttachRequestResponse::BountyTooLow;
  }

  if (self->id == destination->id) {
    return AttachRequestResponse::Self;
  }

  if (self->frequency != destination->frequency) {
    return AttachRequestResponse::Frequency;
  }

  if (destination->ship >= 8) {
    return AttachRequestResponse::Spectator;
  }

  ShipSettings& dest_settings = connection.settings.ShipSettings[destination->ship];

  if (dest_settings.TurretLimit == 0) {
    return AttachRequestResponse::TargetShipNotAttachable;
  }

  size_t turret_count = GetTurretCount(*destination);
  if (turret_count >= dest_settings.TurretLimit) {
    return AttachRequestResponse::TooManyTurrets;
  }

  if (IsAntiwarped(*self, true)) {
    return AttachRequestResponse::Antiwarped;
  }

  connection.SendAttachRequest(destination->id);

  if (ship_controller) {
    ship_controller->ship.fake_antiwarp_end_tick = GetCurrentTick() + connection.settings.AntiwarpSettleDelay;
  }

  AttachPlayer(*self, *destination);
  requesting_attach = true;

  return AttachRequestResponse::Success;
}

void PlayerManager::AttachPlayer(Player& requester, Player& destination) {
  requester.attach_parent = destination.id;

  // Fetch or allocate new AttachInfo and append it to destination's children list
  if (!attach_free) {
    attach_free = memory_arena_push_type(&perm_arena, AttachInfo);
    attach_free->next = nullptr;
  }

  AttachInfo* info = attach_free;
  attach_free = attach_free->next;

  info->player_id = requester.id;
  info->next = destination.children;

  destination.children = info;
}

void PlayerManager::OnCreateTurretLink(u8* pkt, size_t size) {
  u16 request_id = *(u16*)(pkt + 1);

  if (size < 5) {
    Player* self = GetSelf();

    if (self) {
      DetachPlayer(*self);
    }

    return;
  }

  u16 destination_id = *(u16*)(pkt + 3);

  Player* requester = GetPlayerById(request_id);

  if (requester && destination_id == kInvalidPlayerId) {
    DetachPlayer(*requester);
  } else {
    Player* destination = GetPlayerById(destination_id);

    if (!requester || !destination) return;

    if (requester->id == player_id) {
      Player* self = GetSelf();

      // If the attach happening was requested (not server controlled), then reduce energy
      if (self && self->attach_parent == destination_id) {
        if (requesting_attach) {
          self->energy = self->energy * 0.333f;
          requesting_attach = false;

          Event::Dispatch(PlayerAttachEvent(*requester, *destination));
        }
        return;
      }
    }

    AttachPlayer(*requester, *destination);
    Event::Dispatch(PlayerAttachEvent(*requester, *destination));

    if (requester->id != player_id) {
      requester->position = destination->position;
      requester->velocity = destination->velocity;
      requester->lerp_velocity = destination->lerp_velocity;
      requester->lerp_time = destination->lerp_time;
    }
  }
}

void PlayerManager::OnDestroyTurretLink(u8* pkt, size_t size) {
  u16 pid = *(u16*)(pkt + 1);

  Player* player = GetPlayerById(pid);

  if (player) {
    Player* self = GetSelf();
    if (self && self->attach_parent == pid && self->enter_delay <= 0.0f) {
      requesting_attach = false;
      connection.SendAttachRequest(kInvalidPlayerId);
    }
    DetachAllChildren(*player);
  }
}

void PlayerManager::DetachPlayer(Player& player) {
  if (player.attach_parent != kInvalidPlayerId) {
    Player* parent = GetPlayerById(player.attach_parent);

    if (player.id == player_id) {
      requesting_attach = false;
      connection.SendAttachRequest(kInvalidPlayerId);
    }

    if (parent) {
      AttachInfo* current = parent->children;
      AttachInfo* prev = nullptr;

      while (current) {
        // Remove player from the parent's list of attached ships and return it to freelist.
        if (current->player_id == player.id) {
          if (prev) {
            prev->next = current->next;
          } else {
            parent->children = current->next;
          }

          current->player_id = kInvalidPlayerId;
          current->next = attach_free;
          attach_free = current;

          break;
        }

        prev = current;
        current = current->next;
      }

      Event::Dispatch(PlayerDetachEvent(player, *parent));
    }

    player.attach_parent = kInvalidPlayerId;
    // Make player not synchronized so they don't appear until a position packet comes in.
    player.timestamp = kInvalidSmallTick;
  }
}

void PlayerManager::DetachAllChildren(Player& player) {
  AttachInfo* current = player.children;

  while (current) {
    AttachInfo* remove = current;
    current = current->next;

    Player* child = GetPlayerById(remove->player_id);
    if (child && child->attach_parent == player.id) {
      child->attach_parent = kInvalidPlayerId;
      // Make player not synchronized so they don't appear until a position packet comes in.
      child->timestamp = kInvalidSmallTick;

      Player* self = GetSelf();

      if (child == self) {
        requesting_attach = false;
        connection.SendAttachRequest(0xFFFF);
      }
    }

    remove->player_id = kInvalidPlayerId;
    remove->next = attach_free;
    attach_free = remove;
  }

  player.children = nullptr;
}

size_t PlayerManager::GetTurretCount(Player& player) {
  AttachInfo* info = player.children;
  size_t count = 0;

  while (info) {
    ++count;
    info = info->next;
  }

  return count;
}

bool PlayerManager::SimulateAxis(Player& player, float dt, int axis, bool extrapolating) {
  float bounce_factor = 16.0f / connection.settings.BounceFactor;
  Map& map = connection.map;

  int axis_flip = axis == 0 ? 1 : 0;
  float radius = connection.settings.ShipSettings[player.ship].GetRadius();
  float previous = player.position.values[axis];

  player.position.values[axis] += player.velocity.values[axis] * dt;
  float delta = player.velocity.values[axis] * dt;

  if (player.lerp_time > 0.0f) {
    float timestep = dt;
    if (player.lerp_time < timestep) {
      timestep = player.lerp_time;
    }

    player.position.values[axis] += player.lerp_velocity.values[axis] * timestep;
    delta += player.lerp_velocity.values[axis] * timestep;
  }

  u16 check = (u16)(player.position.values[axis] + radius);

  if (delta < 0) {
    check = (u16)floorf(player.position.values[axis] - radius);
  }

  s16 start = (s16)(player.position.values[axis_flip] - radius - 1);
  s16 end = (s16)(player.position.values[axis_flip] + radius + 1);

  Vector2f collider_min = player.position.PixelRounded() - Vector2f(radius, radius);
  Vector2f collider_max = player.position.PixelRounded() + Vector2f(radius, radius);

  bool collided = check < 0 || check > 1023;
  for (s16 other = start; other < end && !collided; ++other) {
    if (axis == 0 && map.IsSolid(check, other, player.frequency)) {
      if (BoxBoxIntersect(collider_min, collider_max, Vector2f((float)check, (float)other),
                          Vector2f((float)check + 1, (float)other + 1))) {
        collided = true;
        break;
      }
    } else if (axis == 1 && map.IsSolid(other, check, player.frequency)) {
      if (BoxBoxIntersect(collider_min, collider_max, Vector2f((float)other, (float)check),
                          Vector2f((float)other + 1, (float)check + 1))) {
        collided = true;
        break;
      }
    }
  }

  if (collided) {
    u32 tick = GetCurrentTick();
    // Don't perform a bunch of wall slowdowns so the player doesn't get very slow against walls.
    if (!extrapolating && TICK_DIFF(tick, player.last_bounce_tick) < 1) {
      bounce_factor = 1.0f;
    }

    player.position.values[axis] = previous;

    player.velocity.values[axis] *= -bounce_factor;
    player.velocity.values[axis_flip] *= bounce_factor;

    player.lerp_velocity.values[axis] *= -bounce_factor;
    player.lerp_velocity.values[axis_flip] *= bounce_factor;

    return true;
  }

  return false;
}

void PlayerManager::SimulatePlayer(Player& player, float dt, bool extrapolating) {
  if (!extrapolating && !IsSynchronized(player)) {
    player.velocity = Vector2f(0, 0);
    player.lerp_time = 0.0f;
    return;
  }

  bool x_bounce = SimulateAxis(player, dt, 0, extrapolating);
  bool y_bounce = SimulateAxis(player, dt, 1, extrapolating);

  if (x_bounce || y_bounce) {
    if (!extrapolating) {
      player.last_bounce_tick = GetCurrentTick();
    }
  }

  TileId tile_id = connection.map.GetTileId(player.position);
  if (tile_id == kTileIdWormhole && player.id == this->player_id) {
    float energy_cost = player.energy * 0.8f;

    if (connection.send_damage) {
      WeaponData wd = {WeaponType::Wormhole, 0, 0, 0, 0, 0};
      PushDamage(this->player_id, wd, (int)player.energy, (int)energy_cost);
    }

    this->Spawn(false);
    player.velocity = Vector2f(0, 0);

    if (player.energy > energy_cost) {
      player.energy -= energy_cost;
    } else {
      player.energy = 1;
    }
  }

  player.lerp_time -= dt;
}

bool PlayerManager::IsAntiwarped(Player& self, bool notify) {
  float antiwarp_tiles = connection.settings.AntiWarpPixels / 16.0f;
  float antiwarp_range_sq = antiwarp_tiles * antiwarp_tiles;

  u32 tick = GetCurrentTick();

  if (ship_controller && TICK_GT(ship_controller->ship.fake_antiwarp_end_tick, tick)) {
    return true;
  }

  for (size_t i = 0; i < player_count; ++i) {
    Player* player = players + i;

    if (player->ship == 8) continue;
    if (player->enter_delay > 0.0f) continue;
    if (player->frequency == self.frequency) continue;
    if (!(player->togglables & Status_Antiwarp)) continue;
    if (!radar->InRadarView(player->position)) continue;

    float dist_sq = player->position.DistanceSq(self.position);

    if (dist_sq <= antiwarp_range_sq) {
      return true;
    }
  }

  return false;
}

}  // namespace zero
