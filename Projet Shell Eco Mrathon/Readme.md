# Système de Contrôle pour Véhicule Électrique - Shell Eco-marathon

![Projet](https://img.shields.io/badge/Project-Shell%20Eco--marathon-green)
![Platform](https://img.shields.io/badge/Platform-STM32F4-blue)
![Status](https://img.shields.io/badge/Status-Calculs%20Corrig%C3%A9s-orange)

## 📌 Présentation du Projet
Ce projet consiste en la conception et la réalisation d'une unité centrale de commande (VCU) pour un prototype de véhicule électrique participant au **Shell Eco-marathon**. L'objectif est d'optimiser la gestion énergétique pour maximiser l'autonomie tout en assurant un contrôle précis du moteur Brushless DC (BLDC).

## ⚙️ Spécifications et Dynamique Corrigée
Les calculs dynamiques ont été mis à jour pour refléter une masse totale réelle de **110 kg** (Véhicule 50 kg + Pilote 60 kg).

### Bilan des Forces Corrigé


| Force | Formule | Valeur Corrigée |
| :--- | :--- | :--- |
| **Inertie ($F_1$)** | $M \times \tau$ | **60,50 N** |
| **Roulement ($F_2$)** | $C_{rr} \times M \times g$ | **16,19 N** |
| **Aérodynamique ($F_3$)** | $\frac{1}{2} \rho \cdot S \cdot C_x \cdot V^2$ | **0,96 N** |
| **Gravité ($F_4$)** | $M \cdot g \cdot \sin(2^\circ)$ | **37,66 N** |

### Analyse des Puissances (Vitesse Max : 30 km/h)
* **Puissance de Croisière (Plat) :** ~143 W (Puissance moyenne nécessaire en course).
* **Puissance de Croisière (Pente 2°) :** ~457 W.
* **Puissance de Crête (Accélération + Pente) :** **966 W** à la roue.

**Justification de la motorisation :** Bien que la puissance de croisière sur plat soit faible (~143 W), une puissance de crête de près de **1000 W** est requise pour respecter l'accélération cible (0 à 30 km/h en 15s) sur une pente de 2°. Un moteur de **600 W** pourrait suffire pour optimiser le rendement si les contraintes d'accélération sont assouplies.



## 🛠️ Architecture Matérielle (Hardware)

Le système repose sur une architecture 48V :
* **Microcontrôleur :** STM32F407VGT6 (Cœur Cortex-M4, 168 MHz).
* **Moteur :** BLDC 48V / 1000W avec capteurs à effet Hall.
* **Batterie :** Lithium NMC 46.8V - 20.8Ah (973Wh).
* **Capteurs :** Courant Isolé (ACS758), Température (NTC), Position (Hall).
* **IHM :** Écran tactile Nextion pour l'affichage des données critiques.

## 💻 Architecture Logicielle (Software)
Le firmware est conçu pour un fonctionnement en temps réel avec une consommation minimale.
* **Commande Moteur :** Algorithme de commande trapézoïdale à 6 étapes ("Six-step commutation").
* **Génération PWM :** Implémentation logicielle pour le pilotage séquentiel des phases moteur.
* **Acquisition ADC :** Lecture des capteurs avec moyennage sur 10 échantillons pour stabiliser les mesures.
* **Interface :** UART bidirectionnel pour le dialogue avec l'écran Nextion et le Smart BMS.

## 📂 Structure du Dépôt
* `/Software` : Code source C (STM32CubeIDE).
* `/Calculs` : Bilan des forces et puissances mis à jour.
* `/HMI` : Fichiers projet Nextion Editor.
* `/Simulation` : Modélisation MATLAB/Simulink et résultats aérodynamiques.


---
*Projet réalisé au sein de l'École Nationale Supérieure d'Ingénieurs de Tunis (ENSIT).*
