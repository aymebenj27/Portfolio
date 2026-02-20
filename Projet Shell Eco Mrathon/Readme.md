Système de Contrôle pour Véhicule Électrique - Shell Eco-marathon
📌 Présentation du Projet
Ce projet consiste en la conception et la réalisation d'une unité centrale de commande (VCU) pour un prototype de véhicule électrique participant à la compétition internationale Shell Eco-marathon. L'objectif principal est d'optimiser la gestion énergétique pour maximiser l'autonomie et l'efficacité du véhicule.
+2

Le système centralise les données des capteurs, supervise l'état du véhicule en temps réel et génère les signaux de commande pour un moteur Brushless DC (BLDC) de 1000W.
+1

🚀 Fonctionnalités Principales

Contrôle Moteur : Pilotage précis (vitesse, accélération, sens de rotation) via une commande trapézoïdale à 6 étapes.
+1


Surveillance en Temps Réel : Monitoring de la tension, du courant, de la température et de l'état de charge (SOC).
+1


Sécurité Embarquée : Protection contre les surtensions, surintensités et surchauffes (arrêt d'urgence et coupure via BMS).
+1


Interface Homme-Machine (IHM) : Affichage des données critiques sur écran tactile Nextion.
+1


Gestion de Batterie : Intégration d'un Smart BMS pour l'équilibrage des cellules et la sécurité du pack Lithium-Ion.
+1

🛠️ Architecture Matérielle (Hardware)
Le système repose sur une architecture 48V:


Microcontrôleur : STM32F407VGT6 (Cortex-M4, 168 MHz).
+2


Moteur : BLDC 48V / 1000W avec capteurs à effet Hall intégrés.
+1


Batterie : Lithium NMC 46.8V - 20.8Ah (973Wh).
+1

Capteurs :

Courant : ACS758 (CJMCU-758).

Température : Sonde NTC / LM35.
+1

Position : Capteurs Hall intégrés au moteur.


Puissance : Onduleur DC/AC (Pont en H MOSFET).
+1

💻 Architecture Logicielle (Software)
Le logiciel est développé en C via l'écosystème STM32Cube.


Algorithme de commande : Génération de signaux PWM (logiciels ou via timers) pour la commutation séquentielle des phases du moteur.
+1


Traitement de données : Moyennage des lectures ADC pour une meilleure stabilité des mesures.

Communication :

UART pour l'écran Nextion et le BMS.

Interface graphique conçue sous Nextion Editor.

📊 Résultats et Simulations

Modélisation : Utilisation de MATLAB/Simulink pour simuler le comportement du moteur et valider les lois de commande.
+1


Tests Pratiques : Validation de la linéarité du capteur de courant et des séquences de commutation via analyseur logique.
+1

📂 Structure du Dépôt
/Software : Code source STM32 (main.c, interruptions, gestion ADC/UART).

/HMI : Fichiers projet Nextion Editor (.hmi).

/Simulation : Modèles MATLAB/Simulink.

/Docs : Cahier des charges et rapport technique.


Développé par : Ben Jemaa Aymen, Hammami Nour, Jammeli Mohamed Yassine 


Institution : École Nationale Supérieure d'Ingénieurs de Tunis (ENSIT)