/*
 * przyciski.c
 *
 *  Created on: 10.09.2018
 *      Author: Konasz
 */

#include "przyciski.h"

void info_przyciski(uint32_t wynik) {
	switch (wynik) {

	case 12289:
	case 14337:
		send("Pierwszy     \n\r", 15);
		break;

	case 12290:
	case 14338:
		send("Drugi        \n\r", 15);
		break;

	case 12291:
	case 14339:
		send("Trzeci       \n\r", 15);
		break;

	case 12292:
	case 14340:
		send("Czwarty      \n\r", 15);
		break;

	case 12293:
	case 14341:
		send("Piaty        \n\r", 15);
		break;

	case 12294:
	case 14342:
		send("Szosty       \n\r", 15);
		break;

	case 12295:
	case 14343:
		send("Siodmy       \n\r", 15);
		break;

	case 12296:
	case 14344:
		send("Osmy         \n\r", 15);
		break;

	case 12297:
	case 14345:
		send("Dziewiaty    \n\r", 15);
		break;

	case 12288:
	case 14336:
		send("Zero         \n\r", 15);
		break;

	case 12348:
	case 14396:
		send("Telegazeta   \n\r", 15);
		break;

	case 12300:
	case 14348:
		send("Wlacznik     \n\r", 15);
		break;

	case 12320:
	case 14368:
		send("Kanal +      \n\r", 15);
		break;

	case 12321:
	case 14369:
		send("Kanal -      \n\r", 15);
		break;

	case 12304:
	case 14352:
		send("Vol +        \n\r", 15);
		break;

	case 12305:
	case 14353:
		send("Vol -        \n\r", 15);
		break;

	case 12301:
	case 14349:
		send("Mute         \n\r", 15);
		break;

	default:
		send("Brak wcisniec\n\r", 15);
	}
}
