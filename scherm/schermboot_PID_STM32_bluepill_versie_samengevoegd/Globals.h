#pragma once

namespace Globals {

enum class Menu : uint8_t {
  OFF,
  VOORVLEUGEL,
  ACHTERVLEUGEL,
  BALANS_VOORVLEUGEL,
  DEBUG,
  STARTUP
};

Menu menu;

const float pi = 3.14159265359;

} // namespace Globals