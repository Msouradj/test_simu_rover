#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "rover.h"

float coef_reduction_freq=0.2;
float coef_augmentation_freq=1.3;

float coef_reduction_speed=0.5;
float coef_augmentation_speed=1.5;

float convert_ms_to_kmh(float speed){
	return speed*3.6;
}

float convert_kmh_to_ms(float speed){
	return speed/3.6;
}


void apply_speed(output_controller output, rover * rover_robot){
	rover_robot->roue1.vitesse=output.vitesse_droit;
	rover_robot->roue2.vitesse=output.vitesse_droit;
	rover_robot->roue3.vitesse=output.vitesse_droit;
	rover_robot->roue4.vitesse=output.vitesse_gauche;
	rover_robot->roue5.vitesse=output.vitesse_gauche;
	rover_robot->roue6.vitesse=output.vitesse_gauche;
}


output_controller controller(rover * rover_robot,output_controller output_pre){
	
	output_controller output;
	if(rover_robot->button_emergency){
		output.retour_base=true;
		printf("Bouton emergency : Retour à la base\n");
		return output;
	}
	if(rover_robot->batterie<10){
		printf("Batterie faible : Retour à la base\n");
		output.retour_base=true;
		return output;	
	}
	if( (rover_robot->distance_front < 1.0) ||
		(rover_robot->distance_back < 1.0) ||
		(rover_robot->distance_top < 1.0) ||
		(rover_robot->distance_right < 1.0) ||
		(rover_robot->distance_left < 1.0))
	{
		output.freq_echantillonage=coef_augmentation_freq*output_pre.freq_echantillonage;
		output.vitesse_droit=coef_reduction_speed*output_pre.vitesse_droit;
		output.vitesse_gauche=coef_reduction_speed*output_pre.vitesse_gauche;	

	}
	else{
		output.freq_echantillonage=coef_reduction_freq*output_pre.freq_echantillonage;
		
		// Pour que la vitesse reste inférieur à 2 km/h
		if(coef_augmentation_speed*output_pre.vitesse_droit<=convert_kmh_to_ms(2)){
			output.vitesse_droit=coef_augmentation_speed*output_pre.vitesse_droit;
		}
		else{
			output.vitesse_droit=convert_kmh_to_ms(2);
		}
		// Pour que la vitesse reste inférieur à 2 km/h
		if(coef_augmentation_speed*output_pre.vitesse_gauche<=convert_kmh_to_ms(2)){
			output.vitesse_gauche=coef_augmentation_speed*output_pre.vitesse_gauche;
		}
		else{
			output.vitesse_gauche=convert_kmh_to_ms(2);
		}
	}

	apply_speed(output,rover_robot);

	return output;
}

rover create_rover(int x_init,int y_init,float batterie){
	rover rover_robot;

	// On initialise la vitesse des roues
	rover_robot.roue1.vitesse=convert_kmh_to_ms(2);
	rover_robot.roue2.vitesse=convert_kmh_to_ms(2);
	rover_robot.roue3.vitesse=convert_kmh_to_ms(2);
	rover_robot.roue4.vitesse=convert_kmh_to_ms(2);
	rover_robot.roue5.vitesse=convert_kmh_to_ms(2);
	rover_robot.roue6.vitesse=convert_kmh_to_ms(2);
	
	// On initialise la position initiale
	rover_robot.x=x_init;
	rover_robot.y=x_init;

	// On sauvegarde les positions initiales
	rover_robot.x_init=x_init;
	rover_robot.y_init=y_init;

	// On met des valeurs arbitraires
	rover_robot.distance_front=1000;
	rover_robot.distance_back=20;
	rover_robot.distance_top=10;
	rover_robot.distance_right=510;
	rover_robot.distance_left=554;

	// Au début le bouton emergency n'est pas appuyé
	rover_robot.button_emergency=false;

	return rover_robot;
}

void move_rover(rover * rover_robot,int x,int y){
	rover_robot->x=x;
	rover_robot->y=y;

}

void update_capteur(rover * rover_robot, 
	float distance_front, 
	float distance_back,
	float distance_top,
	float distance_right,
	float distance_left){

	rover_robot->distance_front=distance_front;
	rover_robot->distance_back=distance_back;
	rover_robot->distance_top=distance_top;
	rover_robot->distance_right=distance_right;
	rover_robot->distance_left=distance_left;
}


void button_pressed(rover * rover_robot){
	rover_robot->button_emergency=!rover_robot->button_emergency;
}

void update_rover_batterie(rover * rover_robot,int new_batterie){
	rover_robot->batterie=new_batterie;
}

bool speed_max_control(rover rover_robot){
	return rover_robot.roue1.vitesse<2.0;
}

void print_rover_position(rover rover_robot){
	printf("\nRover position\n");
	printf("x=%d\n",rover_robot.x);
	printf("y=%d\n",rover_robot.y);
}

void print_rover_capteur(rover  rover_robot){
	printf("\nRover capteur\n");
	printf("front=%.2f\n",rover_robot.distance_front);
	printf("back=%.2f\n",rover_robot.distance_back);
	printf("top=%.2f\n",rover_robot.distance_top);
	printf("right=%.2f\n",rover_robot.distance_right);
	printf("left=%.2f\n",rover_robot.distance_left);
}

void print_rover_button_state(rover  rover_robot){
	printf("\nRover button\n");
	if(rover_robot.button_emergency){
		printf("The button is pressed\n");
	}
	else{
		printf("The button is not pressed\n");
	}
}


// int  main(){
// 	//output_controller output;
// 	//output.freq_echantillonage=1;
// 	//output.retour_base=false;
// 	rover rover_robot=create_rover(0,0,100);
	
// 	//output=controller(&rover_robot,output);
// 	print_rover_position(rover_robot);
// 	print_rover_capteur(rover_robot);
// 	print_rover_button_state(rover_robot);

// 	move_rover(&rover_robot,12,15);
// 	update_capteur(&rover_robot,2,8,16,32,64);
// 	button_pressed(&rover_robot);

// 	print_rover_position(rover_robot);
// 	print_rover_capteur(rover_robot);
// 	print_rover_button_state(rover_robot);

// 	button_pressed(&rover_robot);
// 	print_rover_button_state(rover_robot);

// 	return 1;
// }

