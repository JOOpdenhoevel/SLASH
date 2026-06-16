/**
 * The MIT License (MIT)
 * Copyright (c) 2026 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file config.c
 * @brief Stub configuration loader for the slash-emu daemon.
 *
 * This is the placeholder implementation behind the @c config.h interface.  It
 * allocates an empty config and records the requested source path, but does not
 * parse any file contents.  The real parser (libinih-based, mirroring vrtd) is
 * a separate task; keeping the interface stable here lets the rest of the
 * scaffold be built and tested in isolation.
 */

#define _GNU_SOURCE

#include "config.h"

#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "utils.h"

int emu_config_load(const char *path, struct emu_config **config)
{
    _cleanup_(cleanup_configp)
    struct emu_config *cfg = calloc(1, sizeof(*cfg));
    PROPAGATE_ERROR_NULL_LOG(cfg, LOG_ERR, "Failed to allocate config");

    if (path != NULL) {
        cfg->source_path = strdup(path);
        PROPAGATE_ERROR_NULL_LOG(cfg->source_path, LOG_ERR,
                                 "Failed to duplicate config path");
    }

    LOG(LOG_INFO, "Loaded configuration from %s (stub: contents not parsed)",
        path != NULL ? path : "<default>");

    *config = cfg;
    cfg = NULL;

    return 0;
}

void cleanup_config(struct emu_config *config)
{
    if (config == NULL) {
        return;
    }

    free(config->source_path);
    free(config);
}
