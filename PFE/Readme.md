# Conception et Développement d'un Système Automotive Intelligent de Scan Routier et de Suspension Active

![Status](https://img.shields.io/badge/Status-Projet--PFE-blue)
![Platform](https://img.shields.io/badge/Platform-STM32F407ZGT6-orange)
![Tools](https://img.shields.io/badge/Tools-MATLAB%2FSimulink%20|%20AutoCAD-green)

## 📌 Présentation du Projet
Ce Projet de Fin d'Études (réalisé à SAB Robotics) vise à remplacer les systèmes de suspension passifs par une **suspension active intelligente**. Le système est capable d'analyser l'état de la route en temps réel (scan routier) et d'ajuster dynamiquement les amortisseurs pour stabiliser le châssis, notamment lors du franchissement d'obstacles ou dans les virages à forte force centrifuge.

## ⚙️ Caractéristiques Techniques
Le projet repose sur une synergie entre mécanique, électronique et traitement de signal :

* **Scan Routier :** Système de détection d'obstacles par télémétrie laser (Capteur VL53L0X) pour anticiper les irrégularités.
* **Stabilisation Gyrométrique :** Utilisation d'un accéléromètre/gyroscope (MPU6050) pour maintenir l'assiette du véhicule.
* **Modélisation :** Étude cinétique du modèle "Quart de véhicule" (Quarter-Car Model) simulée sous MATLAB/Simulink.

## 🛠️ Architecture Matérielle (Hardware)
* **Unité de Contrôle :** STM32F407ZGT6 (Cortex-M4, 168 MHz).
* **Actionneurs :** Moteurs pas à pas NEMA 14 avec accouplements élastiques en aluminium.
* **Interface :** Écran tactile intelligent USART HMI (Nextion) pour le monitoring en temps réel.
* **Capteurs :** * Capteur de distance laser (LIDAR TOF).
    * Centrale inertielle (IMU).

## 💻 Environnement Logiciel
* **STM32CubeIDE 1.11.2 :** Développement du firmware en C (HAL).
* **MATLAB / Simulink :** Simulation des lois de commande et réponse aux chocs (dos d'âne).
* **USART HMI :** Conception de l'interface utilisateur graphique.
* **Tera Term / Hercules :** Debugging et monitoring des trames série.

## 📂 Structure du Projet
* `/Firmware` : Code source C pour STM32 (gestion PWM, I2C, UART).
* `/Simulation` : Fichiers Simulink (.slx) modélisant le comportement dynamique.
* `/Hardware` : Schémas de câblage et fiches techniques (Annexes).
* `/Design` : Fichiers AutoCAD pour la conception mécanique du banc d'essai.

## 👥 Équipe
* **Étudiants :** Ben Jemaa Aymen & Meziene Narjes.
* **Encadrants Universitaires :** M. Mhamdi Abdelbacet & Mme. Sellami Wafa (ISET Bizerte).
* **Encadrant Professionnel :** M. Hamdi Saber Youssef (SAB Robotics).

---
*Travail réalisé dans le cadre du diplôme de Licence Appliquée en Génie Électrique (Référence ELNI 03.23).*