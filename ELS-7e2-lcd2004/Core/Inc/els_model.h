#ifndef ELS_MODEL_H
#define ELS_MODEL_H

#include <stdint.h>

/* Constants from Arduino sketch (units preserved) */
#define ENC_LINE_PER_REV     1800
#define MOTOR_Z_STEP_PER_REV 1000
#define SCREW_Z              400   /* сотки мм */
#define McSTEP_Z             2
#define MOTOR_X_STEP_PER_REV 300
#define SCREW_X              150   /* сотки мм */
#define McSTEP_X             4

/* Rapid / limit teach (Arduino: MAX_RAPID_MOTION, MIN_RAPID_MOTION, REPEAt) */
#define MAX_RAPID_MOTION     40
#define MIN_RAPID_MOTION     (MAX_RAPID_MOTION + 165)
#define ELS_REPEAT           (McSTEP_Z * 1)
#define REBOUND_X            1500  /* microsteps */
#define REBOUND_Z            1500

#define MIN_FEED             2     /* сотки/оборот */
#define MAX_FEED             25
#define MIN_aFEED            20    /* мм/мин */
#define MAX_aFEED            250

#define PASS_FINISH          1
#define Th                  13300

typedef struct
{
	uint8_t Cs_Div;
	int Cm_Div;
	char Cone_Print[6];
} cone_info_t;

typedef struct
{
	uint8_t Ks_Div_Z;
	int Km_Div_Z;
	uint8_t Ks_Div_X;
	int Km_Div_X;
	char Thread_Print[7];
	float Step;
	uint8_t Pass;
	char Limit_Print[5];
} thread_info_t;

extern const cone_info_t Cone_Info[];
extern const uint32_t TOTAL_CONE;

extern const thread_info_t Thread_Info[];
extern const uint32_t TOTAL_THREADS;

extern const int Cutter_Width_array[];
extern const uint32_t TOTAL_CUTTER_WIDTH;
extern const int Cutting_Width_array[];
extern const uint32_t TOTAL_CUTTING_STEP;

#endif

