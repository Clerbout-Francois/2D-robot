#include <LiquidCrystal_I2C.h> 

#include <Wire.h>

#include "StepperMotor2.h" //appel de fonctions qui permettent de commander le moteur pas
                           //à pas et d'afficher les valeurs mesurées par les capteurs

#include <Keypad.h>

LiquidCrystal_I2C lcd(0x20, 16, 2);

//initialisation du clavier
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

//Variables pour la position
int posy = 0;
int posx = 0;


// joystick
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output



// initialisation des pins moustaches
int pinILS0 = 53;     //butée droite
int pinILS1 = 52;     //butée gauche


//Initialisation des pins moteurs 
 int pin_Rouge1 = 23;   //translation moteur 0
 int pin_Verte1 = 25;   
 int pin_Bleue1 = 27;   
 int pin_Noire1 = 29;
 int pin_Rouge2 = 43;   //rotation alpha moteur 2
 int pin_Verte2 = 45;   
 int pin_Bleue2 = 47;   
 int pin_Noire2 = 49;                   
 int pin_Rouge3 = 33;   //rotation theta moteur 1
 int pin_Verte3 = 35;   
 int pin_Bleue3 = 37;   
 int pin_Noire3 = 39;   



//initialisation de variable continuation
int continuer;
int continuermode;
int continuer2;
int continuermode2;
int continuer3;
int continuermode3;

//Commande des moteurs 
 int temps_demi_pas = 5000; //duree nécessaire entre chaque demi-pas (en microseconde : us)
                            // ne pas descendre en dessous de 700us
 int anglenumerique=0;
 float pasmoteur = PI/100;   //le pas du moteur, 1,8° par pas, ici exprimé en rad
 float anglealpha;
 float angletheta;

// initialisation de la variable erreur
int erreur;

//Longueur de la crémaillère
  int longueurcremaillere = 100;

// Taille des bras
  float l1=23.3;        
  float l2=23.8;
  
// Longueur entre le point origine et final
  float L;

// Definition des variables de position
  float X;  // a définir au clavier
  float Y;  // a definir au clavier
  float Xm1 = l1 + l2;
  float Ym1 = 0;

//Definition de Y minimal
  int d = 5;    //distance entre l'arbre moteur et la fin de la plaque plexiglas
  int e = 0;    // epaisseur du profilé, inutile ici car le moteur est finalement au dessus
  int Ymin = d + e;

// Le pas pour la translation sur la crémaillère
  float pas = 1;
//L'angle pour parcourir 1 cm, =24°
  float angle_tr = (2*PI)/15;     //15 dents pour faire un tour entier
  float angle_translation=(2*angle_tr)/pasmoteur;

//Definition des angles 
  float theta;
  float alpha;
  float theta0 = PI/2;
  float alpha0 = 0;
  float theta_precedent = theta0;
  float alpha_precedent = alpha0;
  float omega;
  float thetamin = 0;
  float thetamax = PI - thetamin;
  float anglealpha_precedent;
  float angletheta_precedent;

// Equation d'un disque (pour la detection d'erreur)
  float equation_disque_1 = pow((X-(l1 + l2)),2) + pow(Y,2);
  float equation_disque_2 = pow((X-(longueurcremaillere + l1 + l2)),2) + pow(Y,2);
  float rayon_carre = pow(l1 + l2,2);


//Variables pour la translation
  int multiplicateur=0;
  int tour;


void setup() {
  Serial.begin(9600);
  pinMode(pin_Rouge1,OUTPUT);
  pinMode(pin_Verte1,OUTPUT);
  pinMode(pin_Bleue1,OUTPUT);
  pinMode(pin_Noire1,OUTPUT);
  pinMode(pin_Rouge2,OUTPUT);
  pinMode(pin_Verte2,OUTPUT);
  pinMode(pin_Bleue2,OUTPUT);
  pinMode(pin_Noire2,OUTPUT);
  pinMode(pin_Rouge3,OUTPUT);
  pinMode(pin_Verte3,OUTPUT);
  pinMode(pin_Bleue3,OUTPUT);
  pinMode(pin_Noire3,OUTPUT);
  
  //Pins moustaches
  pinMode(pinILS0, INPUT_PULLUP);
  pinMode(pinILS1, INPUT_PULLUP);

  lcd.backlight();        //permet d'avoir un écran plus lumineux
}


      // fonctions externes, qu'on appelle dans la fonction principale

//Fonction de detection d'erreur (espace de travail)
int detection_erreur(float X, float Y) {
  erreur = 0;

  if (Y < Ymin){
    erreur = 1;
  }
  else if ((X >= (l1+l2)) && (X<= (longueurcremaillere+l1+l2)) && (Y>(l1+l2))) {
    erreur = 1;
  }
  else if ((X>=0) && (X<=(l1+l2)) && (equation_disque_1 > rayon_carre)) {
    erreur = 1;
  }
  else if ((X>=longueurcremaillere+l1+l2) && (X<=(2*(l1+l2)+longueurcremaillere)) && (equation_disque_2 > rayon_carre)) {
    erreur = 1;
  }
  else if (X<0) {
    erreur = 1;
  }
  else if (X > (2*(l1+l2)+longueurcremaillere)){
    erreur = 1;
  }

  return erreur;
}


// fonction recherche de theta
float recherche_theta(float X,float L,float Xm1,int l1,int l2){

  omega = acos ((X-Xm1)/L);
  theta = acos ((pow(l1,2) + pow(L,2) - pow(l2,2))/(2*l1*L)) + omega;
  return theta;

}

// fonction de recherche de alpha
float recherche_alpha(float L,int l1,int l2){

  alpha = acos ((pow(l1,2) + pow(l2, 2) - pow (L,2))/(2*l1*l2)) - PI;
  return alpha;

}


// fonction entrée de position
int position(){
  int pos = 0;
  int nvpos = 0;
 suitex:
  char customKey = customKeypad.getKey();
  if (customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == '*' || customKey == '#' && customKey){     // ces caractères ne sont pas pris en compte
    goto suitex;
  }
  if (customKey != 'D' && customKey){       //tant qu'on ne valide pas par D, on reste dans la fonction
    
    nvpos = customKey - '0';                // variable de position
    pos = pos*10 + nvpos;
    goto suitex;
  }
  if (customKey='D' && customKey){        // quand on valide par D, on sort et renvoie la position rentrée
    return (pos);
  }
  goto suitex;
}

//fonction choix 1 ou 2
int getc(){
  entre :
  char customKey = customKeypad.getKey();
  if ( customKey == '1' || customKey == '2'){
    return (customKey-'0');
  }
  goto entre;
}

      //  FONCTIONS D'AFFICHAGE 
      
void messageinitial(){
  // Afficher un message sur le LCD
  lcd.setCursor(0, 0);
  lcd.print("Je suis TiCaK et");
  lcd.setCursor(0,1);
  lcd.print("je vais t'aider");
  delay(3000);
}

void affichagemode(){       //choix du mode
  lcd.setCursor(0, 0);
  lcd.print("MODE ? 1.Clavier");
  lcd.setCursor(0, 1);
  lcd.print("2.Joy 3.JoyDyna");
}

void affichagesaisieclavier(){    //entrées des coordonées
    lcd.setCursor(0, 0);
    lcd.print("Entrez X et Y");
    lcd.setCursor(0, 1);
    lcd.print("Validez par D");
}

void affichageposition(){     //affichage des coordonées rentrées
    lcd.setCursor(0, 0);
    lcd.print("X = ");
    lcd.setCursor(0, 1);
    lcd.print("Y = ");
}

void affichagetransition(){
  lcd.setCursor(0, 0);
  lcd.print("Position validee");
  lcd.setCursor(0, 1);
  lcd.print("Travail en cours");
}

void affichageangle(){      //affichage des angles
  lcd.setCursor(0, 0);
  lcd.print("Theta = ");
  lcd.setCursor(0, 1);
  lcd.print("Alpha = ");
}

void affichagejoystick(){       //affichage des instructions pour utiliser le joystick
  lcd.setCursor(0, 0);
  lcd.print("Bras1 horizontal");
  lcd.setCursor(0, 1);
  lcd.print("Bras2 vertical");
  delay (2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Appuyer pour ");
  lcd.setCursor(0, 1);
  lcd.print("la translation");
  delay (2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Appuyer sur D ");
  lcd.setCursor(0, 1);
  lcd.print("pour valider");
}

void affichagejoystickdynamique(){       //affichage des instructions pour utiliser le joystick dynamique
  lcd.setCursor(0, 0);
  lcd.print("Appuyer pour ");
  lcd.setCursor(0, 1);
  lcd.print("la translation");
  delay (2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Appuyer sur D ");
  lcd.setCursor(0, 1);
  lcd.print("pour valider");
}

void stopticak(){
  lcd.setCursor(0, 0);
  lcd.print("Voulez-vous");
  lcd.setCursor(0, 1);
  lcd.print("arreter ? ");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("arreter ? ");
  lcd.setCursor(0, 1);
  lcd.print("1.Oui 2.Non");
}

void stopmode(){
  lcd.setCursor(0, 0);
  lcd.print("Voulez-vous ");
  lcd.setCursor(0, 1);
  lcd.print("changer le mode");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("changer le mode");
  lcd.setCursor(0, 1);
  lcd.print("1.Oui 2.Non");
  
}

void arretticak(){      //affichage de fin
  lcd.setCursor(0, 0);
  lcd.print("TiCaK se rendort");
  lcd.setCursor(0, 1);
  lcd.print("Merci pour tout");
  delay(3000);
}



void loop() {       //BOUCLE DE PROGRAMME
  lcd.init();     // démarrer l'afficheur  
  messageinitial(); //message de départ
  lcd.clear();
      
choixmode :           //choix du mode
  affichagemode();
  char customKey = customKeypad.getKey();
  if ( customKey == '1'){
    goto modeclavier;         //clavier
  }
  if (customKey == '2'){
    goto modejoy;             //joystick
  }
  if ( customKey == '3'){
    goto modejoydyna;         //joystick dynamique
  }
  if (customKey){             //autre touche que 1, 2 ou 3
    lcd.clear();
    lcd.print("mauvaise touche");
    delay(1000);
    lcd.clear ();
  }
  goto choixmode;
modeclavier :
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    Clavier");       //affichage du mode
  delay(1000);
  lcd.clear();

debut1 : 
  lcd.clear();
  affichagesaisieclavier();       //rentrer la position voulue
  X= position();
  Y = position();
  lcd.clear();
  affichageposition();            //affichage des coordonnées rentrées
  lcd.setCursor(4, 0);
  lcd.print(X);
  lcd.setCursor(4, 1);
  lcd.print(Y);
  delay(2000);
  lcd.clear();
  affichagetransition();
  delay(2000);
  

  L = sqrt(pow((X-Xm1),2) + pow (Y-Ym1,2));   //determination de la longueur entre le point d'origine et le point final
  erreur = detection_erreur(X,Y);
  if (erreur == 1) {                              // detection d'erreur
    goto erreur1;
  }
  else {
    multiplicateur = 0;
    while (L>l1+l2) {       // translation 
      if (X>Xm1) {
        tour = 1;     // pour savoir dans quel sens le moteur doit tourner pour la translation plus loin dans le programme
        Xm1 = Xm1 + pas;
        multiplicateur = multiplicateur + 1;    //le nombre de pas que le moteur doit effectuer
      }
      else {
        tour = 2;
        Xm1 = Xm1 - pas;
        multiplicateur = multiplicateur + 1;        
      }
      
      L = sqrt(pow((X-Xm1),2) + pow (Y-Ym1,2));
    }
    theta = recherche_theta(X,L,Xm1,l1,l2);     //appel des fonctions de recherche
    alpha = recherche_alpha(L,l1,l2);
    if ((theta > thetamax) && (theta <= (2*PI))){     //changement des angles pour respecter les contraintes de la conception
      theta = theta + alpha;
      alpha = - alpha;
    }
    lcd.clear();
    affichageangle();           //affichage de theta et alpha
    lcd.setCursor(9, 0);
    lcd.print(theta);
    lcd.setCursor(9, 1);
    lcd.print(alpha);
  }
  


  //Dans la boucle qui suit, nous commandons le moteur  pas à pas connecté 
  // Pour cela nous utilisons la fonction commandePas définie dans la bibliothèque StepperMotor2

// rotation de alpha

anglealpha = abs((2*alpha)/pasmoteur);        //on multiplie par 2 car on a des demis pas
anglealpha_precedent = abs((2*alpha_precedent)/pasmoteur);
if (alpha > alpha_precedent){
  if (anglealpha > anglealpha_precedent){

   for (int angle_numerique = anglealpha_precedent ; angle_numerique <= anglealpha ; angle_numerique++)       
   {       
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);         //moteur 2
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = anglealpha ; angle_numerique <= anglealpha_precedent ; angle_numerique++)       
    {       
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);         //moteur 2
        delayMicroseconds(temps_demi_pas);

    }
  }
}
else{
  if (anglealpha > anglealpha_precedent){
    for (int angle_numerique = anglealpha; angle_numerique >= anglealpha_precedent ; angle_numerique--)
   {     
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);      //moteur 2
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = anglealpha_precedent; angle_numerique >= anglealpha ; angle_numerique--)
    {     
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);      //moteur 2
        delayMicroseconds(temps_demi_pas);

    }
  }
   
} 

//rotation de theta
angletheta = abs((2*theta)/pasmoteur);
angletheta_precedent = abs((2*theta_precedent)/pasmoteur);

if (theta > theta_precedent){
  if (angletheta > angletheta_precedent){

   for (int angle_numerique = angletheta_precedent ; angle_numerique <= angletheta ; angle_numerique++)       
   {
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);         //moteur 1
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = angletheta ; angle_numerique <= angletheta_precedent ; angle_numerique++)       
    {    
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);         //moteur 1
        delayMicroseconds(temps_demi_pas);

    }
  }
}
else{
  if (angletheta > angletheta_precedent){
    for (int angle_numerique = angletheta; angle_numerique >= angletheta_precedent ; angle_numerique--)
   {       
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);      //moteur 1
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = angletheta_precedent; angle_numerique >= angletheta ; angle_numerique--)
    {      
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);      //moteur 1
        delayMicroseconds(temps_demi_pas);
        //Serial.println("Miel Pops");

    }
  }
   
}
//translation 
if (tour == 1){
   for (int angle_numerique=0; angle_numerique <(multiplicateur*angle_translation) ; angle_numerique++)
        {         
            commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false);      //moteur 0
            delayMicroseconds(temps_demi_pas);

        }
}
else if (tour == 2){
  for (int angle_numerique=(multiplicateur*angle_translation); angle_numerique >=0 ; angle_numerique--)
        {  
          commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false);       //moteur 0
          delayMicroseconds(temps_demi_pas);
        }
}
goto fin;

erreur1:
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("  Erreur");
  lcd.setCursor(0,1);
  lcd.print("Inatteignable");
  theta = theta_precedent;
  alpha = alpha_precedent;

  
fin:
  delay(2000);
  lcd.clear();
  stopticak();
choixcontinuer :
  continuer= getc();
  // arreter ? OUI 1    NON 2
  if ( continuer == 1){ 
    goto final;
  }
  if ( continuer == 2){
    goto cont;
  }
  goto choixcontinuer;
  
  
cont : 
  lcd.clear();
  stopmode();
  continuermode= getc();
  // Changer de mode ? OUI 1    NON 2
choixcontinuermode :
  if ( continuermode == 1){ 
    goto choixmode;
  }
  if ( continuermode == 2){
    theta_precedent = theta;
    alpha_precedent = alpha;
    goto debut1;
  }
  goto choixcontinuermode;
  
modejoy :
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print( "    Joystick");
  delay(2000);
debut2:
  lcd.clear();
  affichagejoystick();
    rotationjoy:
    customKey = customKeypad.getKey();
    if (customKey == 'D'){    //valider la position
      goto finjoystick;
    }
    if (analogRead(X_pin)  >= 800)    //appui sur le bouton central
    {
      delay (500);
      goto translation;
    }
    //rotations de theta et alpha
    else if ((analogRead(X_pin)>520) && (analogRead(X_pin)<800)){
      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)
      {      
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false); 
        delayMicroseconds(temps_demi_pas);
      }
    }
    else if(analogRead(X_pin)<490){
      for (int angle_numerique=0; angle_numerique <=10 ; angle_numerique++)
      {      
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false); 
        delayMicroseconds(temps_demi_pas);
      }
    }
    else if (analogRead(Y_pin)>520){
      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)
      {      
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
        delayMicroseconds(temps_demi_pas);
      }
    }
    else if(analogRead(Y_pin)<490){
      for (int angle_numerique=0; angle_numerique <=10 ; angle_numerique++)
      {      
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
        delayMicroseconds(temps_demi_pas);
      }
    }
    goto rotationjoy;
  
  delay (3000);
  // translation quand on clique
translation:
// Programme pour arreter temporairement les moteurs
//  if (digitalRead(pinILS0)==1){
//    goto blocagemoteurjoystickdroit;
//  }
//  else if (digitalRead(pinILS1)==1){
//    goto blocagemoteurjoystickgauche;
//  }
  customKey = customKeypad.getKey();
  if (customKey == 'D'){
    goto finjoystick;
  }
  else if (analogRead(X_pin) >= 800){
    delay (500);
    goto rotationjoy;
  }
   else if ((analogRead(X_pin)>520) && (analogRead(X_pin)<800) && digitalRead(pinILS0) == 0){
      for (int angle_numerique=0; angle_numerique <=10 ; angle_numerique++)
      {      
        commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false); 
        delayMicroseconds(temps_demi_pas);
      }
   }
   else if(analogRead(X_pin)<490 && digitalRead(pinILS1) == 0){
      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)
      {      
        commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false); 
        delayMicroseconds(temps_demi_pas);
      }
   }
   goto translation;

  delay (1000);
  
finjoystick:  
  lcd.clear();
  stopticak();
choixcontinuer2 :
  continuer2= getc();
  // arreter ? OUI 1    NON 2
  if ( continuer2 == 1){ 
    goto final2;
  }
  if ( continuer2 == 2){
    goto cont2;
  }
  goto choixcontinuer2;

cont2 : 
  lcd.clear();
  stopmode();
  continuermode2 = getc();
  // Changer de mode ? OUI 1    NON 2
choixcontinuermode2 :
  if ( continuermode2 == 1){ 
    goto choixmode;
  }
  if ( continuermode2 == 2){
    goto debut2;
  }
  goto choixcontinuermode2;
  
modejoydyna:
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print( "   Joystick ");
  lcd.setCursor(0,1);
  lcd.print("  dynamique");
  delay(2000);
  lcd.clear();
debut3:
  lcd.clear();
  //ajouter un affichage
rotationjoydyna:
    customKey = customKeypad.getKey();
    if (customKey == 'D'){    //valider la position
      goto finjoystickdynamique;
    }
    if (analogRead(X_pin)  >= 800)    //appui sur le bouton central
    {
      delay (500);
      goto translationdyna;
    }
    //rotations de theta et alpha
    else if ((analogRead(X_pin)>520) && (analogRead(X_pin)<800)){
      for (int angle_numerique=0; angle_numerique <=10 ; angle_numerique++)         //theta diminue, alpha augmente
      {      
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);
        commandeStep((10 - angle_numerique),pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
        delayMicroseconds(temps_demi_pas);
      }
//      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)       //alpha augmente
//      {      
//        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
//        delayMicroseconds(temps_demi_pas);
//      }
    }
    else if(analogRead(X_pin)<490){
      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)        //theta augmente, alpha diminue
      {      
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);
        commandeStep((10 - angle_numerique),pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
        delayMicroseconds(temps_demi_pas);
      }
//      for (int angle_numerique=0; angle_numerique >=10 ; angle_numerique++)       //alpha diminue
//      {      
//        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
//        delayMicroseconds(temps_demi_pas);
//      }
    }
    else if (analogRead(Y_pin)>520){
      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)         //theta et alpha augmentent
      {      
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
        //delayMicroseconds(temps_demi_pas);
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false); 
        delayMicroseconds(temps_demi_pas);
      }
    }
    else if(analogRead(Y_pin)<490){
      for (int angle_numerique=0; angle_numerique <=10 ; angle_numerique++)         //theta et alpha diminue
      {      
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false); 
        //delayMicroseconds(temps_demi_pas);
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false); 
        delayMicroseconds(temps_demi_pas);
      }
    }
    goto rotationjoydyna;
  
  delay (3000);
  // translation quand on clique
translationdyna:
  customKey = customKeypad.getKey();
  if (customKey == 'D'){
    goto finjoystickdynamique;
  }
  else if (analogRead(X_pin) >= 800){
    delay (500);
    goto rotationjoydyna;
  }
   else if ((analogRead(X_pin)>520) && (analogRead(X_pin)<800) && digitalRead(pinILS0) == 0){
      for (int angle_numerique=0; angle_numerique <=10 ; angle_numerique++)
      {      
        commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false); 
        delayMicroseconds(temps_demi_pas);
      }
   }
   else if(analogRead(X_pin)<490 && digitalRead(pinILS1) == 0){
      for (int angle_numerique=10; angle_numerique >=0 ; angle_numerique--)
      {      
        commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false); 
        delayMicroseconds(temps_demi_pas);
      }
   }
   goto translationdyna;

  delay (1000);
  
finjoystickdynamique:  
  lcd.clear();
  stopticak();
choixcontinuer3 :
  continuer3= getc();
  // arreter ? OUI 1    NON 2
  if ( continuer3 == 1){ 
    goto final2;
  }
  if ( continuer3 == 2){
    goto cont3;
  }
  goto choixcontinuer3;

cont3 : 
  lcd.clear();
  stopmode();
  continuermode3 = getc();
  // Changer de mode ? OUI 1    NON 2
choixcontinuermode3 :
  if ( continuermode3 == 1){ 
    goto choixmode;
  }
  if ( continuermode3 == 2){
    goto debut3;
  }
final:

  lcd.clear();
  arretticak();
    // retour à la position d'initialisation
alpha = alpha0;
theta = theta0;

// rotation de alpha

anglealpha = abs((2*alpha)/pasmoteur);
anglealpha_precedent = abs((2*alpha_precedent)/pasmoteur);
if (alpha > alpha_precedent){
  if (anglealpha > anglealpha_precedent){

   for (int angle_numerique = anglealpha_precedent ; angle_numerique <= anglealpha ; angle_numerique++)       
   {       
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);         //moteur 2
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = anglealpha ; angle_numerique <= anglealpha_precedent ; angle_numerique++)       
    {       
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);         //moteur 2
        delayMicroseconds(temps_demi_pas);

    }
  }
}
else{
  if (anglealpha > anglealpha_precedent){
    for (int angle_numerique = anglealpha; angle_numerique >= anglealpha_precedent ; angle_numerique--)
   {       
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);      //moteur 2
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = anglealpha_precedent; angle_numerique >= anglealpha ; angle_numerique--)
    {       
        commandeStep(angle_numerique,pin_Rouge2,pin_Verte2,pin_Bleue2,pin_Noire2,false);      //moteur 2
        delayMicroseconds(temps_demi_pas);

    }
  }
   
} 

//rotation de theta
angletheta = abs((2*theta)/pasmoteur);
angletheta_precedent = abs((2*theta_precedent)/pasmoteur);

if (theta > theta_precedent){
  if (angletheta > angletheta_precedent){

   for (int angle_numerique = angletheta_precedent ; angle_numerique <= angletheta ; angle_numerique++)       
   {       
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);         //moteur 1
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = angletheta ; angle_numerique <= angletheta_precedent ; angle_numerique++)       
    {       
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);         //moteur 1
        delayMicroseconds(temps_demi_pas);

    }
  }
}
else{
  if (angletheta > angletheta_precedent){
    for (int angle_numerique = angletheta; angle_numerique >= angletheta_precedent ; angle_numerique--)
   {       
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);      //moteur 1
        delayMicroseconds(temps_demi_pas);

   }
  }
  else{
    for (int angle_numerique = angletheta_precedent; angle_numerique >= angletheta ; angle_numerique--)
    {       
        commandeStep(angle_numerique,pin_Rouge3,pin_Verte3,pin_Bleue3,pin_Noire3,false);      //moteur 1
        delayMicroseconds(temps_demi_pas);
        //Serial.println("Miel Pops");

    }
  }
   
} 

//translation au point x initial

while (digitalRead(pinILS1) == 0){
  for (int angle_numerique = angle_translation; angle_numerique >= 0 ; angle_numerique++){  
  
       commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false);       //moteur 0
       delayMicroseconds(temps_demi_pas);
  }
  
}



        


lcd.clear();
goto arretmoteur;

final2:
  lcd.clear();
  arretticak();
  delay(3000);
  lcd.clear();
  goto arretmoteur;

//blocagemoteurjoystickdroit:            //boucle sans fin sauf si appui de l'utilisateur sur la touche * du pad
//for (int angle_numerique=3*angle_translation; angle_numerique >=0 ; angle_numerique--)
//      {      
//        //Serial.println("Froosties");
//        commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false); 
//        delayMicroseconds(temps_demi_pas);
//      }
//suiteblocagedroite:
//customKey = customKeypad.getKey();
//  if (customKey == '*' && customKey){    
//      goto modejoy;
//      delay(500);
//  }   
//  else{
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print( "Moteur en ");
//  lcd.setCursor(0, 1);
//  lcd.print( "butee mecanique");
//  delay (2000);
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print( "Appuyez sur *");
//  lcd.setCursor(0, 1);
//  lcd.print( "pour reprise");
//  delay (2000);
//  }
//  goto suiteblocagedroite;
//
//blocagemoteurjoystickgauche:            //boucle sans fin sauf si appui de l'utilisateur sur la touche * du pad
//for (int angle_numerique=0; angle_numerique >=3*angle_translation ; angle_numerique--)
//      {      
//        //Serial.println("CocoPops");
//        commandeStep(angle_numerique,pin_Rouge1,pin_Verte1,pin_Bleue1,pin_Noire1,false); 
//        delayMicroseconds(temps_demi_pas);
//      }
//suiteblocagegauche:
//customKey = customKeypad.getKey();
//  if (customKey == '*' && customKey){    
//      goto modejoy;
//      delay(500);
//  }   
//  else{
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print( "Moteur en ");
//  lcd.setCursor(0, 1);
//  lcd.print( "butee mecanique");
//  delay (2000);
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print( "Appuyez sur *");
//  lcd.setCursor(0, 1);
//  lcd.print( "pour reprise");
//  delay (2000);
//  }
//  goto suiteblocagegauche;
//
//blocagemoteurclavier:                 //boucle sans fin sauf si appui de l'utilisateur sur la touche * du pad
//customKey = customKeypad.getKey();
//  if (customKey == '*'){    
//      goto modeclavier;
//  }   
//  else{
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print( "Moteurs en ");
//  lcd.setCursor(0, 1);
//  lcd.print( "butee mecanique");
//  delay (2000);
//  }
//  goto blocagemoteurclavier;


arretmoteur:
int a = 2;
  while (a<20)   //boucle sans fin
  {
    
  }
  
}
