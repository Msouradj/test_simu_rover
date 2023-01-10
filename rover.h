#ifndef ROVER_H
#define ROVER_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>


typedef struct roue{
float vitesse; // en m/s
} roue;

typedef struct output_controller{
float vitesse_droit; // en m/s
float vitesse_gauche; // en m/s
float freq_echantillonage;
bool retour_base;
} output_controller;

typedef struct rover{
roue roue1;
roue roue2;
roue roue3;
roue roue4;
roue roue5;
roue roue6;
float distance_front;
float distance_back;
float distance_top;
float distance_right;
float distance_left;
int pression;
float temperature;
bool button_emergency;
int x;
int y;
int x_init;
int y_init;
int batterie;
} rover;

extern float convert_ms_to_kmh(float speed);
extern float convert_kmh_to_ms(float speed);
extern void apply_speed(output_controller output, rover *rover_robot);
extern output_controller controller(rover * rover_robot,output_controller output_pre);
extern rover create_rover (int x_init,int y_init,float batterie);
extern void move_rover(rover *rover_robot,int x,int y);
extern void update_capteur(rover * rover_robot,float distance_front,float distance_back,float distance_top,float distance_right,float distance_left);
extern void button_pressed(rover * rover_robot);
extern void update_rover_batterie(rover * rover_robot,int new_batterie);
extern bool speed_max_control (rover  rover_robot);
extern void print_rover_button_state(rover  rover_robot);
extern void print_rover_position(rover rover_robot);
extern void print_rover_capteur(rover  rover_robot);

#endif