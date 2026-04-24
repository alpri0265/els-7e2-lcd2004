#ifndef ELS_MENU_H
#define ELS_MENU_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "LiquidCrystalRus.h"

// Constants from the original sketch
#define MAX_FEED 25
#define MIN_FEED 2
#define MAX_aFEED 250
#define MIN_aFEED 20

typedef enum
{
	ELS_MODE_FEED = 1,
	ELS_MODE_AFEED,
	ELS_MODE_THREAD,
	ELS_MODE_CONE_L,
	ELS_MODE_CONE_R,
	ELS_MODE_SPHERE,
	ELS_MODE_TACHO,
	ELS_MODE_RESERVE
} els_mode_t;

typedef enum { ELS_SUB_INT = 1, ELS_SUB_MAN, ELS_SUB_EXT } els_submode_t;

class MenuManager
{
public:
	MenuManager(LiquidCrystalRus& lcd);
	
	void init();
	void update();
	void render();

	// Setters for external inputs (ADC, Buttons, Joystick, Switches)
	void setAdcRaw10(uint16_t adc10);
	void handleKeyEvent(uint8_t key); // e.g. 0=Select, 1=Up, 2=Down, 3=Left, 4=Right
	
	// Read hardware state (you can map these to actual STM32 pins in main.cpp)
	// For now, these can be set externally or read directly if we pass pin numbers
	void setModeSubmode(uint8_t mode_val, uint8_t submode_val);

private:
	LiquidCrystalRus& lcd;

	// Menu state
	els_mode_t mode;
	els_submode_t sub_thread;
	els_submode_t sub_feed;
	els_submode_t sub_afeed;
	els_submode_t sub_cone;
	els_submode_t sub_sphere;

	uint8_t select_menu; // 0..2

	// Menu variables
	int ap;               
	int pass_total;       
	int pass_nr;          
	int pass_fin;         
	int thr_pass_summ;    
	long pass_total_sphr; 
	uint8_t total_tooth;  
	uint8_t current_tooth;
	uint8_t thread_step;  
	uint8_t cone_step;    
	long sph_r_mm;        
	long bar_r_mm;        
	uint8_t cutter_step;
	uint8_t cutting_step;
	int enc_pos;          
	uint32_t duration;    

	// ADC state
	uint16_t adc_feed;
	uint32_t sum_adc;
	uint16_t adc_array[16];
	uint8_t adc_idx;

	// Derived values
	uint16_t feed_mm;   
	uint16_t afeedback_mm; 

	// Positions
	long x_pos;
	long z_pos;

	// Flags
	bool err_1;
	bool err_2;
	bool complete;

	// Helper functions
	void writePadded(const char* s);
	void beepStub();
	void keySelectPressed();
	void keyUpPressed();
	void keyDownPressed();
	void keyLeftPressed();
	void keyRightPressed();
};

#endif
