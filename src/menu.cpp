#include "menu.h"
#include <stdio.h>
#include <string.h>

// Custom characters for LCD
static const uint8_t chr_left[8]   = {0x00,0x04,0x08,0x1F,0x08,0x04,0x00,0x00};
static const uint8_t chr_right[8]  = {0x00,0x04,0x02,0x1F,0x02,0x04,0x00,0x00};
static const uint8_t chr_up[8]     = {0x00,0x04,0x0E,0x15,0x04,0x04,0x00,0x00};
static const uint8_t chr_down[8]   = {0x00,0x04,0x04,0x15,0x0E,0x04,0x00,0x00};
static const uint8_t chr_degree[8] = {0x0C,0x12,0x12,0x0C,0x00,0x00,0x00,0x00};
static const uint8_t chr_diam[8]   = {0x01,0x0E,0x13,0x15,0x19,0x0E,0x10,0x00};

MenuManager::MenuManager(LiquidCrystal& lcd_ref) : lcd(lcd_ref)
{
	mode = ELS_MODE_FEED;
	sub_feed = ELS_SUB_MAN;
	sub_thread = ELS_SUB_MAN;
	sub_afeed = ELS_SUB_MAN;
	sub_cone = ELS_SUB_MAN;
	sub_sphere = ELS_SUB_MAN;
	select_menu = 0;

	ap = 0;
	pass_total = 1;
	pass_nr = 1;
	pass_fin = 0;
	thr_pass_summ = 0;
	pass_total_sphr = 1;
	total_tooth = 1;
	current_tooth = 1;
	thread_step = 0;
	cone_step = 0;
	sph_r_mm = 1000;
	bar_r_mm = 0;
	cutter_step = 4;
	cutting_step = 2;
	enc_pos = 0;
	duration = 0;

	adc_feed = 0;
	sum_adc = 0;
	for (uint8_t i = 0; i < 16; i++) adc_array[i] = 0;
	adc_idx = 0;
	feed_mm = MAX_FEED;
	afeedback_mm = MAX_aFEED;

	x_pos = 0;
	z_pos = 0;

	err_1 = false;
	err_2 = false;
	complete = false;
}

void MenuManager::init()
{
	lcd.begin(20, 4);
	lcd.createChar(1, (uint8_t*)chr_left);
	lcd.createChar(2, (uint8_t*)chr_right);
	lcd.createChar(3, (uint8_t*)chr_up);
	lcd.createChar(4, (uint8_t*)chr_down);
	lcd.createChar(5, (uint8_t*)chr_degree);
	lcd.createChar(6, (uint8_t*)chr_diam);
}

void MenuManager::writePadded(const char* s)
{
	size_t len = s ? strlen(s) : 0;
	for (size_t i = 0; i < 20; i++)
	{
		if (s && i < len) lcd.print(s[i]);
		else lcd.print(' ');
	}
}

void MenuManager::beepStub()
{
	// TODO: implement beep (digitalWrite to BEEPER pin)
}

void MenuManager::keySelectPressed()
{
	switch (mode)
	{
		case ELS_MODE_FEED:
			select_menu = (select_menu == 0) ? 1 : 0;
			beepStub();
			break;
		case ELS_MODE_AFEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
		case ELS_MODE_THREAD:
		case ELS_MODE_SPHERE:
			select_menu = (select_menu + 1) % 3;
			beepStub();
			break;
		case ELS_MODE_TACHO:
			select_menu = (select_menu == 0) ? 1 : 0;
			beepStub();
			break;
		default:
			break;
	}
}

void MenuManager::keyUpPressed()
{
	// Example for Feed Mode
	if (mode == ELS_MODE_FEED) {
		if (select_menu == 0) {
			if (ap < 500) { ap += 10; beepStub(); }
			else if (ap < 900) { ap += 50; beepStub(); }
		} else if (select_menu == 1) {
			x_pos = 0;
			beepStub();
		}
	}
}

void MenuManager::keyDownPressed()
{
	if (mode == ELS_MODE_FEED) {
		if (select_menu == 0) {
			if (ap > 500) { ap -= 50; beepStub(); }
			else if (ap > 0) { ap -= 10; beepStub(); }
		} else if (select_menu == 1) {
			x_pos = 0;
			beepStub();
		}
	}
}

void MenuManager::keyLeftPressed()
{
	if (mode == ELS_MODE_FEED) {
		if (select_menu == 0) {
			if (pass_total > 1) { pass_total--; beepStub(); }
		} else if (select_menu == 1) {
			z_pos = 0;
			beepStub();
		}
	}
}

void MenuManager::keyRightPressed()
{
	if (mode == ELS_MODE_FEED) {
		if (select_menu == 0) {
			if (pass_total < 99) { pass_total++; beepStub(); }
		} else if (select_menu == 1) {
			z_pos = 0;
			beepStub();
		}
	}
}

void MenuManager::handleKeyEvent(uint8_t key)
{
	// 0=Sel, 1=Up, 2=Down, 3=Left, 4=Right
	switch(key) {
		case 0: keySelectPressed(); break;
		case 1: keyUpPressed(); break;
		case 2: keyDownPressed(); break;
		case 3: keyLeftPressed(); break;
		case 4: keyRightPressed(); break;
	}
	render();
}

void MenuManager::setAdcRaw10(uint16_t adc10)
{
	if (adc10 > 1023) adc10 = 1023;

	if ((adc10 > adc_feed + 4) || (adc10 + 4 < adc_feed))
	{
		adc_idx++;
		if (adc_idx > 15) adc_idx = 0;
		sum_adc -= adc_array[adc_idx];
		adc_array[adc_idx] = adc10;
		sum_adc += adc10;
		adc_feed = sum_adc / 16;
	}

	if (mode == ELS_MODE_FEED)
	{
		uint16_t feed_new = MAX_FEED - ((MAX_FEED - MIN_FEED + 1) * adc_feed) / 1024;
		if (feed_new != feed_mm)
		{
			feed_mm = feed_new;
			render();
		}
	}
}

void MenuManager::update()
{
	// In the future: handle reading physical inputs here,
	// and call handleKeyEvent accordingly.
}

void MenuManager::render()
{
	char buf[21];

	if (err_1)
	{
		lcd.setCursor(0, 0); writePadded("WARNING");
		lcd.setCursor(0, 1); writePadded("");
		lcd.setCursor(0, 2); writePadded("LIMIT SWITCHES N/A");
		lcd.setCursor(0, 3); writePadded("");
		return;
	}
	if (err_2)
	{
		lcd.setCursor(0, 0); writePadded("MOVE CARRIAGE TO");
		lcd.setCursor(0, 1); writePadded("");
		lcd.setCursor(0, 2); writePadded("HOME POSITION");
		lcd.setCursor(0, 3); writePadded("");
		return;
	}
	if (complete)
	{
		lcd.setCursor(0, 0); writePadded("");
		lcd.setCursor(0, 1); writePadded("CYCLE COMPLETE");
		lcd.setCursor(0, 2); writePadded("");
		lcd.setCursor(0, 3); writePadded("");
		return;
	}

	if (mode == ELS_MODE_FEED)
	{
		if (select_menu == 0)
		{
			lcd.setCursor(0, 0);
			lcd.print("SYNC ");
			
			if (sub_feed == ELS_SUB_INT) { lcd.setCursor(11, 0); lcd.print(" INTERNAL"); }
			else if (sub_feed == ELS_SUB_MAN) { lcd.setCursor(11, 0); lcd.print(" MANUAL  "); }
			else if (sub_feed == ELS_SUB_EXT) { lcd.setCursor(11, 0); lcd.print(" EXTERNAL"); }

			lcd.setCursor(0, 1);
			snprintf(buf, sizeof(buf), "FEED mm/rev: %d.%02d", feed_mm/100, feed_mm%100);
			writePadded(buf);

			lcd.setCursor(0, 2);
			if (sub_feed == ELS_SUB_MAN)
				snprintf(buf, sizeof(buf), "TOTAL PASSES:   %2d", pass_total);
			else
				snprintf(buf, sizeof(buf), "PASSES LEFT:    %2d", pass_total - pass_nr + 1);
			writePadded(buf);

			lcd.setCursor(0, 3);
			snprintf(buf, sizeof(buf), "DOC/PASS:     %d.%01d", ap/100, ap%100);
			writePadded(buf);
		}
		else if (select_menu == 1)
		{
			lcd.setCursor(0, 0); writePadded("");
			lcd.setCursor(0, 1); writePadded("");
			
			lcd.setCursor(0, 2);
			snprintf(buf, sizeof(buf), "X: %s%3ld.%02d mm", (x_pos < 0) ? "-" : " ", abs(x_pos/100), abs(x_pos%100));
			writePadded(buf);

			lcd.setCursor(0, 3);
			snprintf(buf, sizeof(buf), "Z: %s%3ld.%02d mm", (z_pos < 0) ? "-" : " ", abs(z_pos/100), abs(z_pos%100));
			writePadded(buf);
		}
	}
	// TODO: Add other modes (Thread, aFeed, Cone, Sphere, etc.) from Print.ino
}
