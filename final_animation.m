clear all
close all

%% On initialise la figure du bras robotisé

% On crée une comparaison pour la boucle infinie
arretcomparaison = 0;

% Les longueurs des bras et de la crémaillère sont à indiquer en
% centimètres

% Il faudra mettre une condition sur Y avec les caractéristiques d
% (distance entre crémaillère et rail) et e (épaisseur rail)
% Condition sur Y
d = 0;
e = 0;
Ymin = d + e;



% L'origine de la zone de travail se situe au milieu de l'axe de
% translation (au milieu de la crémaillère)

% On définit la longueur des bras l1 et l2
l1 = 15;
l2 = 15;

% On définit la longueur de la crémaillère
longueurcremaillere = 100;

% On représente les bras avec les coordonnées des points
Xm1 = 0;
Xm1initial = Xm1;
Ym1 = 0;

% Définition des axes 
axis ([-80,80,0,40]);

% Initialisation du robot
theta0 = pi/2;
thetainitial = theta0;
alpha0 = 0;
alphainitial = alpha0;

bras1 = line([Xm1, Xm1 + cos(theta0)*l1],[Ym1, sin(theta0)*l1], 'Color','r');
bras2 = line([Xm1 + cos(theta0)*l1, Xm1 + cos(theta0)*l1 + cos(theta0 + alpha0)*l2],[sin(theta0)*l1, sin(theta0)*l1 + sin(theta0 + alpha0)*l2]);

% On définit la valeur de chaque point
Xm2 = Xm1 + cos(theta0)*l1;
Ym2 = sin(theta0)*l1;
Xp = Xm1 + cos(theta0)*l1 + cos(theta0 + alpha0)*l2;
Yp = sin(theta0)*l1 + sin(theta0 + alpha0)*l2;

% Définition du pas de variation de la position de Xm1 et pour
% l'incrémentation des angles
pas = 0.05; % Attention !! Augmenter la valeur du pas réduit la précision finale, on utilise ce pas pour la variation de Xm1
pastemporel = 0.05; % C'est un pas temporel, il s'exprime en seconde, il servira pour le PID


% On définit thetamin et thetamax
thetamin = 0;
thetamax = pi - thetamin;

% La zone de travail du robot a la forme d'un rectangle et de deux quarts
% de disque
% Comme on vérifie la condition sur Yp positif, on peut travailler
% directement avec l'équation de chaque disque

% Valeur du carré du rayon de chaque disque
Rcarre = power((l1 + l2), 2);

%% Boucle principale (réalisation du mouvement entre la position précédente et le nouveau point P à atteindre)

% Arrêt ou non du système
prompt0 = 'Arrêt du robot ? Indiquez 0 pour OUI et 1 pour NON';
arretacomparer = input(prompt0);
while arretacomparer ~= arretcomparaison
    
% Il nous faut la position du point P, c'est l'utilisateur qui va la choisir
prompt1 = 'Entrez la coordonnée Xp du point P à atteindre avec le bras robotisé';
prompt2 = 'Entrez la coordonnée Yp du point P à atteindre avec le bras robotisé';
Xp = input(prompt1);
Yp = input(prompt2);


% Condition sur Yp
prompt3 = 'La valeur de Yp est trop faible, veuillez rentrer une valeur de Yp strictement supérieure à Ymin constructeur';
while Yp < Ymin 
    Yp = input(prompt3);
end


% Equation du disque du côté positif
equationdisquepos = (power(Xp - (longueurcremaillere*0.5), 2)) + (power(Yp - 0, 2));
% Equation du disque du côté négatif 
equationdisqueneg = (power(Xp + (longueurcremaillere*0.5), 2)) + (power(Yp - 0, 2));

% On définit la longueur L entre M1 et P
L = sqrt(power((Xp-Xm1),2) + power((Yp-Ym1),2));

%Initialisation de l'erreur
stop = 0;
% Calcul de la faisabilité du problème
prompt4 = 'Le point P n est pas atteignable';
if (-(longueurcremaillere)/(2)) <= Xp && Xp <= ((longueurcremaillere)/(2)) && Yp > (l1 + l2)
    disp(prompt4);
    stop = 1;
elseif ((longueurcremaillere/(2))) <= Xp && Xp <= (((longueurcremaillere)/(2)) + (l1 + l2)) && equationdisquepos > Rcarre
    disp(prompt4);
    stop = 1;
elseif ((-longueurcremaillere/(2)) - (l1 + l2)) <= Xp && Xp <= ((-longueurcremaillere)/(2)) && equationdisqueneg > Rcarre
    disp(prompt4);
    stop = 1;
elseif Xp > (((longueurcremaillere)/(2)) + (l1 + l2))
    disp(prompt4);
    stop = 1;
elseif Xp < ((-(longueurcremaillere)/(2)) - (l1 + l2))
    disp(prompt4);
    stop = 1;
else
    while L > (l1 +l2)
    if Xp > Xm1
        Xm1 = Xm1 + pas;
    else
        Xm1 = Xm1 - pas;
    end
    L = sqrt(power((Xp-Xm1),2) + power((Yp-Ym1),2));
    end
end

% On calcule ensuite les angles theta et alpha correspondant au problème
 
% On crée ensuite oméga l'angle entre l'horizontale et la droite (M1P)
omega = acos ((Xp-Xm1)/L);
 
% On détermine les valeurs de theta et alpha
if stop==0
    theta = acos ((power(l1,2) + power(L,2) - power(l2,2))/(2*l1*L)) + omega;
    alpha = acos ((power(l1,2) + power(l2, 2) - power (L,2))/(2*l1*l2)) - pi;
else
    theta=theta0;
    alpha=alpha0;
end

% Condition sur theta et alpha
if (theta > thetamax) && (theta <= (2*pi))
    theta = theta + alpha;
    alpha = - alpha;
end


% Détermination du PID pour la translation (on ne travaille qu'en gain
% proportionnel, soit avec Kp)
Kpm1 = 10; % Determiné arbitrairement après des tests, pour Xp = 25 c'est bien
deltaxm1 = abs(Xm1 - Xm1initial);
Pidm1 = Kpm1 * deltaxm1 * pastemporel;
precisionxm1 = 0.01; % Attention, plus la précision est grande, moins le pointeur sera précis
% Rendre le système plus précis augmente le temps de réponse

% Boucle de variation de la position de Xm1
while deltaxm1 >= precisionxm1
        if stop==1
            break
        end
        % Mise à jour des coordonnées des points
        if Xm1 > Xm1initial
            set (bras1, 'XData',[Xm1initial + Pidm1, Xm1initial + cos(thetainitial)*l1 + Pidm1]);
            set (bras2, 'XData',[Xm1initial + cos(thetainitial)*l1 + Pidm1, Xm1initial + cos(thetainitial)*l1 + cos(thetainitial + alphainitial)*l2 + Pidm1]);
            Xm1initial = Xm1initial + Pidm1;
            deltaxm1 = abs(Xm1 - Xm1initial);
            Pidm1 = Kpm1 * deltaxm1 * pastemporel;
            % On rafraichit la figure
            drawnow;
        else
            set (bras1, 'XData',[Xm1initial - Pidm1, Xm1initial + cos(thetainitial)*l1 - Pidm1]);
            set (bras2, 'XData',[Xm1initial + cos(thetainitial)*l1 - Pidm1, Xm1initial + cos(thetainitial)*l1 + cos(thetainitial + alphainitial)*l2 - Pidm1]);
            Xm1initial = Xm1initial - Pidm1;
            deltaxm1 = abs(Xm1 - Xm1initial);
            Pidm1 = Kpm1 * deltaxm1 * pastemporel;
            % On rafraichit la figure
            drawnow;
        end
end


% Calcul des coordonnées du point M2
%Xm2 = Xm1 + cos(theta)*l1;
%Ym2 = sin(theta)*l1;

% Détermination du PID pour la rotation d'angle theta (on ne travaille qu'en gain proportionnel, selon
% avec Kp)
Kpm2 = 10; % Determiné arbitrairement après des tests
deltaxm2 = abs(theta - thetainitial);
Pidm2 = Kpm2 * deltaxm2 * pastemporel;
precisionxm2 = 0.01; % Attention, plus la précision est grande, moins le pointeur sera précis
% Rendre le système plus précis augmente le temps de réponse

% Détermination du PID pour la rotation d'angle alpha (on ne travaille qu'en gain proportionnel, selon
% avec Kp)
Kpp = 10; % Determiné arbitrairement après des tests
deltap = abs(alpha - alphainitial);
Pidp = Kpp * deltap * pastemporel;
precisionp = 0.01; % Attention, plus la précision est grande, moins le pointeur sera précis
% Rendre le système plus précis augmente le temps de réponse


% Boucle de rotation (de la position d'initialisation à la position voulue)

    
if alpha > alphainitial
    while deltap >= precisionp
        if stop==1
            break
        end
        % Mise à jour des coordonnées du point P
        %set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial + Pidm2)*l1], 'YData',[0, sin(thetainitial + Pidm2)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial)*l1, Xm1initial + cos(thetainitial)*l1 + cos(thetainitial + alphainitial + Pidp)*l2], 'YData',[sin(thetainitial)*l1, sin(thetainitial)*l1 + sin(thetainitial + alphainitial + Pidp)*l2]);
        alphainitial = alphainitial + Pidp;
        deltap = abs(alpha - alphainitial);
        Pidp = Kpp * deltap * pastemporel;
        % On rafraichit la figure
        drawnow;
    end
    %alphainitial = alpha;

    
    % Mise à jour des coordonnées du point M2
    if theta > thetainitial
        while deltaxm2 >= precisionxm2
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point M2
        set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial + Pidm2)*l1], 'YData',[0, sin(thetainitial + Pidm2)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2)*l1, Xm1initial + cos(thetainitial + Pidm2)*l1 + cos(thetainitial + Pidm2 + alphainitial + Pidp)*l2], 'YData',[sin(thetainitial + Pidm2)*l1, sin(thetainitial + Pidm2)*l1 + sin(thetainitial + Pidm2 + alphainitial + Pidp)*l2]);
        thetainitial = thetainitial + Pidm2;
        deltaxm2 = abs(theta - thetainitial);
        Pidm2 = Kpm2 * deltaxm2 * pastemporel;
        % On rafraichit la figure
        drawnow;        
        end
        %thetainitial = theta;

    else
        while deltaxm2 >= precisionxm2
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point M2
        set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial - Pidm2)*l1], 'YData',[0, sin(thetainitial - Pidm2)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial - Pidm2)*l1, Xm1initial + cos(thetainitial - Pidm2)*l1 + cos(alphainitial + Pidp + thetainitial - Pidm2)*l2], 'YData',[sin(thetainitial - Pidm2)*l1, sin(thetainitial - Pidm2)*l1 + sin(alphainitial + Pidp + thetainitial - Pidm2)*l2]);
        thetainitial = thetainitial - Pidm2;
        deltaxm2 = abs(theta - thetainitial);
        Pidm2 = Kpm2 * deltaxm2 * pastemporel;
        % On rafraichit la figure
        drawnow;
        end
        %alphainitial = alpha;

    end
else
    while deltap >= precisionp
        if stop==1
            break
        end
        % Mise à jour des coordonnées du point P
        %set (bras1, 'XData',[Xm1initial,Xm1initial + cos(thetainitial - Pidm2)*l1], 'YData',[0, sin(thetainitial - Pidm2)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial)*l1, Xm1initial + cos(thetainitial)*l1 + cos(thetainitial + alphainitial - Pidp)*l2], 'YData',[sin(thetainitial)*l1, sin(thetainitial)*l1 + sin(thetainitial + alphainitial - Pidp)*l2]);
        alphainitial = alphainitial - Pidp;
        deltap = abs(alpha - alphainitial);
        Pidp = Kpp * deltap * pastemporel;
        % On rafraichit la figure
        drawnow;
    end
    %alphainitial = alpha;

        
    % Mise à jour des coordonnées du point M2
    if theta > thetainitial
        while deltaxm2 >= precisionxm2
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point M2
        set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial + Pidm2)*l1], 'YData',[0, sin(thetainitial + Pidm2)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2)*l1, Xm1initial + cos(thetainitial + Pidm2)*l1 + cos(alphainitial - Pidp + thetainitial + Pidm2)*l2], 'YData',[sin(thetainitial + Pidm2)*l1, sin(thetainitial + Pidm2)*l1 + sin(alphainitial - Pidp + thetainitial + Pidm2)*l2]);
        thetainitial = thetainitial + Pidm2;
        deltaxm2 = abs(theta - thetainitial);
        Pidm2 = Kpm2 * deltaxm2 * pastemporel;
        % On rafraichit la figure
        drawnow;
        end
        %thetainitial = theta;

    else
        while deltaxm2 >= precisionxm2
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point M2
        set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial - Pidm2)*l1], 'YData',[0, sin(thetainitial - Pidm2)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial - Pidm2)*l1, Xm1initial + cos(thetainitial - Pidm2)*l1 + cos(alphainitial - Pidp + thetainitial - Pidm2)*l2], 'YData',[sin(thetainitial - Pidm2)*l1, sin(thetainitial - Pidm2)*l1 + sin(alphainitial - Pidp + thetainitial - Pidm2)*l2]);
        thetainitial = thetainitial - Pidm2;
        deltaxm2 = abs(theta - thetainitial);
        Pidm2 = Kpm2 * deltaxm2 * pastemporel;
        % On rafraichit la figure
        drawnow;
        end
        %thetainitial = theta;

    end
    
end


arretacomparer = input(prompt0);
end

%% Arrêt du bras
promptstop = 'Arrêt du robot en cours';
promptarret = 'Retour à la position d initialisation du robot';
disp(promptstop);
disp(promptarret);

% Détermination du PID pour la translation de retour à l'équilibre (on ne travaille qu'en gain proportionnel, selon
% avec Kp)
Xm1 = 0;
Kpm1 = 10; % Determiné arbitrairement après des tests, pour Xp = 25 c'est bien
deltaxm1 = abs(Xm1 - Xm1initial);
Pidm1 = Kpm1 * deltaxm1 * pastemporel;
precisionxm1 = 0.01; % Attention, plus la précision est grande, moins le pointeur sera précis
% Rendre le système plus précis augmente le temps de réponse


% Boucle de variation de la position de Xm1
while deltaxm1 > precisionxm1
        if stop==1
            break
        end
        % Mise à jour des coordonnées des points
        if Xm1 > Xm1initial
            set (bras1, 'XData',[Xm1initial + Pidm1, Xm1initial + cos(thetainitial)*l1 + Pidm1]);
            set (bras2, 'XData',[Xm1initial + cos(thetainitial)*l1 + Pidm1, Xm1initial + cos(thetainitial)*l1 + cos(thetainitial + alphainitial)*l2 + Pidm1]);
            Xm1initial = Xm1initial + Pidm1;
            deltaxm1 = abs(Xm1 - Xm1initial);
            Pidm1 = Kpm1 * deltaxm1 * pastemporel;
            % On rafraichit la figure
            drawnow;
        else
            set (bras1, 'XData',[Xm1initial - Pidm1, Xm1initial + cos(thetainitial)*l1 - Pidm1]);
            set (bras2, 'XData',[Xm1initial + cos(thetainitial)*l1 - Pidm1, Xm1initial + cos(thetainitial)*l1 + cos(thetainitial + alphainitial)*l2 - Pidm1]);
            Xm1initial = Xm1initial - Pidm1;
            deltaxm1 = abs(Xm1 - Xm1initial);
            Pidm1 = Kpm1 * deltaxm1 * pastemporel;
            % On rafraichit la figure
            drawnow;
        end
end

% Réinitialisation du robot
thetafinal = theta0;
alphafinal = alpha0;


% Détermination du PID pour la rotation d'angle theta (on ne travaille qu'en gain proportionnel, selon
% avec Kp)
Kpm2f = 10; % Determiné arbitrairement après des tests
deltaxm2f = abs(thetafinal - thetainitial);
Pidm2f = Kpm2f * deltaxm2f * pastemporel;
precisionxm2f = 0.01; % Attention, plus la précision est grande, moins le pointeur sera précis
% Rendre le système plus précis augmente le temps de réponse

% Détermination du PID pour la rotation d'angle alpha (on ne travaille qu'en gain proportionnel, selon
% avec Kp)
Kppf = 10; % Determiné arbitrairement après des tests
deltapf = abs(alphafinal - alphainitial);
Pidpf = Kppf * deltapf * pastemporel;
precisionpf = 0.01; % Attention, plus la précision est grande, moins le pointeur sera précis
% Rendre le système plus précis augmente le temps de réponse

% Boucle de rotation (de la position actuelle à la position d'initialisation, de réinitialisation)   
if thetafinal > thetainitial
    while deltaxm2f >= precisionxm2f
        if stop==1
            break
        end
        % Mise à jour des coordonnées du point M2
        set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial + Pidm2f)*l1], 'YData',[0, sin(thetainitial + Pidm2f)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2f)*l1,Xm1initial + cos(thetainitial + Pidm2f)*l1 + cos(thetainitial + alphainitial + Pidm2f)*l2], 'YData',[sin(thetainitial + Pidm2f)*l1, sin(thetainitial + Pidm2f)*l1 + sin(thetainitial + alphainitial + Pidm2f)*l2]);
        thetainitial = thetainitial + Pidm2f;
        deltaxm2f = abs(thetafinal - thetainitial);
        Pidm2f = Kpm2f * deltaxm2f * pastemporel;
        % On rafraichit la figure
        drawnow;
    end
    %thetainitial = thetafinal;

    
    % Mise à jour des coordonnées du point P
    if alphafinal > alphainitial
        while deltapf >= precisionpf
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point M2
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2f)*l1, Xm1initial + cos(thetainitial + Pidm2f)*l1 + cos(alphainitial + Pidpf + thetainitial + Pidm2f)*l2], 'YData',[sin(thetainitial + Pidm2f)*l1, sin(thetainitial + Pidm2f)*l1 + sin(alphainitial + Pidpf + thetainitial + Pidm2f)*l2]);
        alphainitial = alphainitial + Pidpf;
        deltapf = abs(alphafinal - alphainitial);
        Pidpf = Kppf * deltapf * pastemporel;
        % On rafraichit la figure
        drawnow;        
        end
        %alphainitial = alphafinal;

    else
        while deltapf >= precisionpf
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point P
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2f)*l1, Xm1initial + cos(thetainitial + Pidm2f)*l1 + cos(alphainitial + Pidpf + thetainitial + Pidm2f)*l2], 'YData',[sin(thetainitial + Pidm2f)*l1, sin(thetainitial + Pidm2f)*l1 + sin(alphainitial + Pidpf + thetainitial + Pidm2f)*l2]);
        alphainitial = alphainitial - Pidpf;
        deltapf = abs(alphafinal - alphainitial);
        Pidpf = Kppf * deltapf * pastemporel;
        % On rafraichit la figure
        drawnow;
        end
        %alphainitial = alphafinal;

    end
else
    while deltaxm2f >= precisionxm2f
        if stop==1
            break
        end
        % Mise à jour des coordonnées du point M2
        set (bras1, 'XData',[Xm1initial, Xm1initial + cos(thetainitial - Pidm2f)*l1], 'YData',[0, sin(thetainitial - Pidm2f)*l1] );
        set (bras2, 'XData',[Xm1initial + cos(thetainitial - Pidm2f)*l1, Xm1initial + cos(thetainitial - Pidm2f)*l1 + cos(thetainitial + alphainitial - Pidm2f)*l2], 'YData',[sin(thetainitial - Pidm2f)*l1, sin(thetainitial - Pidm2f)*l1 + sin(thetainitial + alphainitial - Pidm2f)*l2]);
        thetainitial = thetainitial - Pidm2f;
        deltaxm2f = abs(thetafinal - thetainitial);
        Pidm2f = Kpm2f * deltaxm2f * pastemporel;
        % On rafraichit la figure
        drawnow;
    end
    %thetainitial = thetafinal;

        
    % Mise à jour des coordonnées du point P
    if alphafinal > alphainitial
        while deltapf >= precisionpf
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point M2
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2f)*l1, Xm1initial + cos(thetainitial + Pidm2f)*l1 + cos(alphainitial + Pidp + thetainitial + Pidm2f)*l2], 'YData',[sin(thetainitial + Pidm2f)*l1, sin(thetainitial + Pidm2f)*l1 + sin(alphainitial + Pidpf + thetainitial + Pidm2f)*l2]);
        alphainitial = alphainitial + Pidpf;
        deltapf = abs(alphafinal - alphainitial);
        Pidpf = Kppf * deltapf * pastemporel;
        % On rafraichit la figure
        drawnow;
        end
        %alphainitial = alphafinal;

    else
        while deltapf >= precisionpf
            if stop==1
                break
            end
        % Mise à jour des coordonnées du point P
        set (bras2, 'XData',[Xm1initial + cos(thetainitial + Pidm2f)*l1, Xm1initial + cos(thetainitial + Pidm2f)*l1 + cos(alphainitial - Pidpf + thetainitial + Pidm2f)*l2], 'YData',[sin(thetainitial + Pidm2f)*l1, sin(thetainitial + Pidm2f)*l1 + sin(alphainitial - Pidpf + thetainitial + Pidm2f)*l2]);
        alphainitial = alphainitial - Pidpf;
        deltapf = abs(alphafinal - alphainitial);
        Pidpf = Kppf * deltapf * pastemporel;
        % On rafraichit la figure
        drawnow;
        end
        %alphainitial = alphafinal;

    end
end