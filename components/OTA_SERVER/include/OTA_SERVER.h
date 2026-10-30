#pragma once

typedef uint8_t(*ota_mode_provider_t)(void);

void otaServerStart(void);
void otaServerSetModeProvider(ota_mode_provider_t provider);
