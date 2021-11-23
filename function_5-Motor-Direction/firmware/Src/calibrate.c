/*
 * calibrate.c
 *
 *  Created on: 19 nov. 2021
 *      Author: carol
 */

#include "calibrate.h"

/* Private define ------------------------------------------------------------*/

#define STRAIGHT 50
#define LEFT_MAX_SPEED_STEERING 10 // Left steering max speed
#define RIGHT_MAX_SPEED_STEERING 90 // Right steering max speed

#define pin_blue_user_button GPIO_PIN_13

#define MAX(a,b) ((a)>(b) ? (a) : (b))

/* Private variables ---------------------------------------------------------*/

int right_ok = 0;
int left_ok = 0;
int centre_ok = 0;

int capt_L = 0;
int capt_R = 0;
int capt_C = 0;

int diff_LC = 0;
int diff_RC = 0;

int capt_when_straight = 0;
int capt_when_right = 0;

/* Programs ------------------------------------------------------------------*/

/*
 * Verifie si le user button bleu est appuyé
 */
int is_blue_button_pressed(){
	return (!HAL_GPIO_ReadPin(GPIOC, pin_blue_user_button));
}

/*
 *	Performe la calibration du module de direction (action moteur, recuperation valeur capteur)
 */
void calibrate(void){
	if (!right_ok){
		right_ok = 1;
		steering_set_speed(GPIO_PIN_SET, RIGHT_MAX_SPEED_STEERING);
		HAL_Delay(5000);
		capt_R = steering_get_angle();
	}
	if (!left_ok){
		left_ok = 1;
		steering_set_speed(GPIO_PIN_SET, LEFT_MAX_SPEED_STEERING);
		HAL_Delay(5000);
		capt_L = steering_get_angle();
	}
	if (!centre_ok){
		steering_move_with_button();
		if (is_blue_button_pressed()){
			centre_ok = 1;
			capt_C = steering_get_angle();
		}
	}
	/*
	 * POUR LA CALIBRATION :
	 * 0- Dans le main.c, passer le MODE à 0. Builder et passer en mode Debug.
	 * 1- Run et laisser le programme de calibration tourner pendant environ 10 secondes.
	 * 2- Mettre les roues bien alignées pour rouler tout droit avec les boutons
	 * du tableau de bord. Une fois les roues bien alignées, appuyer sur le
	 * bouton bleu de la NucleoF103RB.
	 * 3- Mettre un point d'arrêt à l'endroit indiqué ci-dessous.
	 * 4- Observer les valeurs de capt_when_straight et capt_when_right grâce à
	 * un clic droit puis "Watch Add Expression".
	 * 5- Passer en vue Debug pour observer ces valeurs dans l'onglet Expressions.
	 * 6- Reporter ces valeurs dans les #define du meme nom dans steering.c.
	 * 7- Dans le main.c, passer le MODE à 2 pour reprendre l'activité en mode
	 * normal.
	 * BRAVO, tu as terminé la calibration !
	 */
	if (centre_ok){
		diff_LC = capt_L - capt_C;
		diff_RC = capt_C - capt_R;
		capt_when_straight = MAX(diff_LC, diff_RC);
		capt_when_right = capt_C - capt_when_straight;

		// Mettre un point d'arret ICI pour la calibration

		steering_set_position(GPIO_PIN_SET, STRAIGHT);
	}

}

