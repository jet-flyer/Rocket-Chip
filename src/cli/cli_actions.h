// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_CLI_ACTIONS_H
#define ROCKETCHIP_CLI_ACTIONS_H

#include "cli/cli_engine.h"

namespace rc {
namespace cli {

void run_action(ActionId act);
void print_help(const Item* table, size_t n, MenuId menu);
void print_prompt(MenuId menu);

}  // namespace cli
}  // namespace rc

#endif
