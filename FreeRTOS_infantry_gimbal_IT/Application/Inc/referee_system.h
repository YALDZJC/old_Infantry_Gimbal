#ifdef REFEREE_GLOBALS
#define REFEREE_EXTERN
#else
#define REFEREE_EXTERN extern
#endif

typedef struct{
	uint16_t shooter_id1_17mm_cooling_heat;
}ext_power_heat_data_t;

REFEREE_EXTERN ext_power_heat_data_t ext_power_heat_data;

void referee_task( void *pvParameters );
