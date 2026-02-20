# 🐱 HelloKitty – Projet Robot Chat
<img width="810" height="1080" alt="image" src="https://github.com/user-attachments/assets/7b001b46-020f-4a15-b75d-87ee0aa59870" />


### **Les contributeurs :**
-BENJEMAA Aymen
-SOLTANI Ezer
-ROUABAH Serine

## 📚 Table des matières

1. [📜 Présentation générale](#-présentation-générale)
2. [📐 Architecture matérielle](#-architecture-matérielle)
   - [🔌 Schéma système](#-schéma-système)
   - [📌 Pinout STM32](#-pinout-stm32)
   - [🛠️ PCB et design électronique](#-pcb-et-design-électronique)
3. [🧩 Architecture logicielle](#-architecture-logicielle)
   - [🧱 Couches logicielles](#-couches-logicielles)
   - [🕒 Fonctionnement des tâches FreeRTOS](#-fonctionnement-des-tâches-freertos)
   - [🔄 Synchronisation et priorités](#-synchronisation-et-priorités)
4. [⚙️ Drivers et HAL](#-drivers-et-hal)
5. [🎯 Stratégie comportementale](#-stratégie-comportementale)
6. [📊 Tests et validation](#-tests-et-validation)
7. [🔧 Résultats et perspectives](#-résultats-et-perspectives)

## Présentation générale

HelloKitty est un robot mobile autonome conçu pour évoluer sur une surface plane sans bordure, dans un jeu de poursuite entre plusieurs robots. Le projet s’inscrit dans le cadre du module Systèmes Électroniques Avancés de l’ENSEA, et vise à couvrir l’ensemble du cycle de développement embarqué : de la conception du PCB à l’implémentation logicielle temps réel, en passant par la stratégie comportementale.

Le robot est capable de détecter les bords, d’éviter les chutes, de repérer d’autres robots, et de changer de rôle (chat ↔ souris) en fonction des interactions physiques ou visuelles. Le projet met en œuvre des capteurs variés, une architecture logicielle modulaire, et une gestion fine des tâches concurrentes via FreeRTOS.

## **Architecture**  
### **Schéma architectural**  

<img width="927" height="693" alt="schema_projet" src="https://github.com/user-attachments/assets/1b3bfaf2-3be3-44c0-845a-d9920e9071d3" />

#  Architecture du système embarqué STM32G431CBU6

Ce projet repose sur un microcontrôleur **STM32G431CBU6**, intégrant divers capteurs, modules de communication, moteurs et interfaces utilisateur pour créer un système autonome capable de capter, traiter et agir dans un environnement physique.

##  Microcontrôleur central
- **STM32G431CBU6** : cœur du système, gère les communications, le traitement des données et le contrôle des périphériques.

## Alimentation
- **Batterie NiMH 7.2V 1.3Ah** : source principale d’énergie.
- **Régulateurs de tension** :
  - **MP1475DJ-LF-P** : convertit la tension en **5V**.
  - **BU33SD5WG-TR** : convertit en **3.3V** pour les composants sensibles.

## Capteurs et modules
- **4 capteurs TOF (Time-of-Flight)** : connectés via **I2C**, pour mesurer les distances.
- **Accéléromètre ADXL343** : connecté en **I2C**, pour détecter les mouvements.
- **Module Bluetooth** : communication sans fil via **UART**.
- **Lidar YDLIDAR X2** : capteur de télémétrie, connecté en **UART**.

## Horloge et programmation
- **Quartz 16MHz** : fournit une horloge stable au microcontrôleur.
- **STLink SWD** : interface de programmation et débogage.
- **Hclk** : 170Mhz.

## Moteurs et contrôle
- **2 pilotes de moteur ZXBM5210** : reçoivent des signaux **PWM** pour contrôler les moteurs gauche et droit.
- **Moteurs avec encodeurs** : permettent un retour de position et de vitesse.

## 🖱️ Interface utilisateur
- **LED** : sortie **GPIO**, pour signalisation.
- **Bouton utilisateur** : entrée **GPIO**, pour interaction manuelle.
- **Bouton reset** : pour redémarrer le système.

## 🔌 Connexions colorées
- **Rouge** : lignes d’alimentation **5V**
- **Orange** : lignes **3.3V**
- **Violet** : **I2C**
- **Bleu** : **UART**
- **Vert** : **GPIO**
- **Noir** : **PWM**

---

Ce schéma illustre l’interconnexion des modules pour un système embarqué intelligent et réactif.

## Fonctionnement interne du robot

Le robot ne se contente pas d’exécuter des actions simples : son microcontrôleur coordonne en continu l’ensemble des capteurs, moteurs et modules pour produire un comportement cohérent et réactif. Cette section décrit la logique interne qui permet au système de fonctionner de manière autonome.

### Organisation logicielle
Le logiciel embarqué est structuré en plusieurs tâches indépendantes.  
Chaque tâche s’occupe d’un domaine précis : analyse des distances, lecture des chocs, gestion des moteurs ou encore surveillance de l’environnement.  
Cette organisation évite qu’une opération bloque les autres et garantit une réactivité constante.

### Système de décision
Le robot suit une hiérarchie de priorités pour réagir correctement aux événements :
- **Sécurité immédiate** : arrêt ou retrait en cas de danger (vide, obstacle trop proche, choc).
- **Évitement** : choix de la direction la plus dégagée grâce aux données du LiDAR.
- **Déplacement normal** : progression ou patrouille lorsque l’environnement est stable.

Cette logique empêche les comportements incohérents et permet des réactions rapides.

### Gestion dynamique des moteurs
Les moteurs sont ajustés en permanence selon la situation :
- correction de trajectoire,
- adaptation de la vitesse,
- compensation en cas de résistance ou de choc.

Le microcontrôleur calcule ces ajustements en temps réel, tandis que les drivers appliquent les consignes via PWM.

### Fusion des capteurs
Les informations issues des différents capteurs sont combinées pour obtenir une vision plus fiable de l’environnement :
- les ToF surveillent les bords,
- le LiDAR analyse l’espace autour du robot,
- l’accéléromètre détecte les impacts ou blocages.

Cette fusion permet d’anticiper les risques et d’adapter le comportement du robot de manière fluide.

### États internes
Le robot fonctionne comme une machine à états, chacun correspondant à un comportement précis :
- exploration,
- évitement,
- collision détectée,
- danger de chute,
- blocage,
- repos.

Chaque état définit les actions à effectuer et les conditions pour passer à un autre état.

### Indicateurs lumineux
Les LEDs servent de retour visuel pour comprendre l’état du robot :
- clignotement rapide : alerte,
- clignotement lent : attente,
- lumière fixe : fonctionnement normal.

Elles permettent de diagnostiquer rapidement le comportement du robot sans accéder au code.

### Synchronisation des communications
Les différents protocoles (UART, SPI, I2C) fonctionnent en parallèle.  
Pour éviter les conflits, les échanges sont cadencés et certaines lectures sont prioritaires.  
Les interruptions matérielles assurent la prise en charge immédiate des événements critiques.

---

Cette architecture logicielle permet au robot d’être autonome, réactif et capable de s’adapter en temps réel à son environnement.


##  Partie Hardware

Cette section décrit l’architecture matérielle du robot, ses composants électroniques, et les schémas associés.
<img width="983" height="564" alt="image" src="https://github.com/user-attachments/assets/481b72ed-1033-43fa-afc0-7f35f21c6596" />


### 🔌 Schéma global du système
Le système repose sur un microcontrôleur **STM32G431CBU6** qui coordonne les capteurs, les moteurs, les régulateurs et les interfaces utilisateur.


---

### ⚙️ Microcontrôleur et interfaces
Le microcontrôleur est au cœur du système. Il est connecté :
- aux moteurs via des signaux **PWM** et des entrées d’encodeurs,
- aux capteurs via **UART**, **I2C**, et **GPIO**,
- à un **STLink/SWD** pour la programmation et le débogage.

<img width="1078" height="742" alt="image" src="https://github.com/user-attachments/assets/20eed7e6-8ec8-43f0-a155-ee942acf7542" />


---

### 🔋 Alimentation et régulation
Le robot est alimenté par une batterie **NiMH 7.2V**, régulée en deux tensions :
- **5V** via le régulateur **MP1475DJ-LF-P**,
- **3.3V** via le régulateur **BU33SD5WG-TR**.

Ces tensions alimentent les moteurs, le microcontrôleur et les capteurs sensibles.

<img width="1013" height="592" alt="image" src="https://github.com/user-attachments/assets/fa8be56d-729b-44cd-a620-70aa3347fee2" />


---

### 🦾 Pilotes de moteurs
Chaque moteur est contrôlé par un circuit **ZXBM5210-SP**, avec :
- deux entrées **PWM** pour la vitesse et la direction,
- deux sorties vers le moteur (Motor+ / Motor−),
- des entrées d’encodeurs pour le retour de position.

Chaque moteur dispose de son propre driver et de ses propres signaux.

<img width="570" height="256" alt="image" src="https://github.com/user-attachments/assets/a448f5a5-3051-4683-b620-6901f1f0ada8" />

---

### 📡 Capteurs
Le système intègre :
- **4 capteurs TOF** pour la détection de bordure,
- **1 accéléromètre ADXL343** pour les chocs et mouvements,
- **1 LiDAR YDLIDAR X2** pour la cartographie et l’évitement,
- **1 module Bluetooth** pour la communication sans fil.

Tous ces capteurs sont connectés au microcontrôleur via **I2C**, **SPI**, **UART** ou **GPIO**.
<img width="1062" height="625" alt="image" src="https://github.com/user-attachments/assets/a634e98e-761b-432a-a299-bae5318e2d9d" />

---

### 🖱️ Interface utilisateur
Le robot dispose :
- de **LEDs** pour indiquer son état (obstacle, marche, pause…),
- d’un **bouton utilisateur** pour les interactions manuelles,
- d’un **bouton reset** pour redémarrer le système.
##im

---

### 🧩 Organisation des fichiers KiCad
Les schémas sont répartis en plusieurs fichiers :
- `pucontrolleur.kicad.sch` : microcontrôleur et interfaces
- `moteur1.kicad.sch` / `moteur2.kicad.sch` : circuits moteurs
- `regulateurs.kicad.sch` : alimentation
- `capteurs.kicad.sch` : capteurs et communication

---

Cette architecture matérielle permet au robot d’être autonome, réactif et modulaire. Chaque composant est interconnecté pour assurer un fonctionnement fluide et sécurisé.




# 🧩 Architecture logicielle

Cette section détaille l'organisation logicielle du robot, son système d'exploitation temps réel (FreeRTOS), les drivers pour les périphériques matériels, et la logique comportementale qui régit ses actions.

## 🧱 Couches logicielles

Le logiciel embarqué est construit sur une architecture multicouche modulaire pour garantir la maintenabilité, la réactivité et la fiabilité. Le système d'exploitation temps réel **FreeRTOS** est au cœur de cette architecture, orchestrant plusieurs tâches concurrentes qui gèrent des aspects spécifiques du robot.

Les principales couches logicielles sont :
1.  **Couche d'Application (Stratégie) :** La logique de haut niveau qui prend des décisions basées sur les données des capteurs et l'état actuel du robot.
2.  **Couche de Service (Tâches FreeRTOS) :** Des tâches indépendantes qui gèrent des fonctionnalités spécifiques comme la communication, la gestion des moteurs ou la surveillance des capteurs.
3.  **Couche d'Abstraction Matérielle (HAL) :** Les drivers et les bibliothèques HAL de STMicroelectronics qui fournissent une interface standardisée pour interagir avec le matériel du microcontrôleur.
4.  **Couche Matérielle :** Le microcontrôleur STM32G431CBU6 et ses périphériques.

## 🕒 Fonctionnement des tâches FreeRTOS

Ce fichier est le cœur de l'application temps réel. Il initialise les tâches FreeRTOS, les mutex et autres objets de synchronisation.

| Tâche | Priorité | Description |
| :--- | :--- | :--- |
| `vControlTask` | 4 (Haute) | Contrôle les moteurs via PID et exécute la stratégie comportementale. |
| `vSafetyTask` | 3 (Moyenne-Haute) | Surveille les capteurs TOF pour éviter les chutes. |
| `vLidarTask` | 2 (Moyenne) | Traite les données du Lidar pour la détection d'obstacles. |
| `vImuTask` | 1 (Basse) | Lit l'accéléromètre pour détecter les chocs. |

### Tâches des Capteurs et de la Sécurité

-   **`vSafetyTask`**: Cette tâche lit les capteurs TOF pour détecter le vide. Si un risque de chute est détecté, elle prend le contrôle des moteurs (`g_safety_override`) pour effectuer une manœuvre d'évitement.
-   **`vLidarTask`**: Elle traite en continu le flux de données du Lidar pour détecter et suivre les objets environnants. L'objet le plus proche est stocké pour être utilisé par la tâche de stratégie.
-   **`vImuTask`**: Elle surveille l'accéléromètre pour détecter les chocs. Un choc déclenche un changement de rôle (Chat ↔ Souris).

```c
// Extrait de vSafetyTask
void vSafetyTask(void *pvParameters)
{
  for(;;) {
      // Lecture des capteurs TOF
      if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
          TOF_Read_All(dist);
          xSemaphoreGive(xI2C1Mutex);
      }

      // Si un vide est détecté en avant
      if (dist[0] > 200 || dist[1] > 200) {
          g_safety_override = 1; // Prend le contrôle
          // ... Séquence de recul et rotation ...
          g_safety_override = 0; // Rend le contrôle
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}
```

### Tâche de Contrôle Moteur (`vControlTask`)

C'est la tâche la plus complexe. Elle est responsable de :
1.  **Mettre à jour la vitesse** mesurée des moteurs à partir des encodeurs.
2.  **Exécuter la stratégie** (`Strategy_Update()`) pour obtenir les vitesses cibles (linéaire et angulaire).
3.  **Calculer les vitesses cibles** pour chaque roue.
4.  **Exécuter les boucles d'asservissement PID** pour chaque moteur.
5.  **Appliquer la commande PWM** aux drivers des moteurs.

```c
// Extrait de vControlTask
void vControlTask(void *pvParameters)
{
    // Initialisation des PIDs et de l'odométrie
    PID_Init(...);

    for(;;)
    {
        // 1. Mesure de la vitesse
        Motor_UpdateSpeed(&hMotor1, dt);
        Motor_UpdateSpeed(&hMotor2, dt);

        if (g_safety_override == 0) {
            // 2. Exécution de la stratégie
            Strategy_Update();

            // 3. Calcul des vitesses des roues
            float target_rad_L = (v_lin - (v_ang * half_track)) / radius;
            float target_rad_R = (v_lin + (v_ang * half_track)) / radius;

            // 4. Calcul de la commande PID
            float pwm_L = PID_Compute(&pid_vel_left,  target_rad_L, speed_L);
            float pwm_R = PID_Compute(&pid_vel_right, target_rad_R, speed_R);

            // 5. Application de la commande
            Motor_SetSpeed(&hMotor2, -pwm_L);
            Motor_SetSpeed(&hMotor1, pwm_R);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}
```

## 🔄 Synchronisation et priorités

La gestion des tâches concurrentes est cruciale pour la stabilité du système.

### Priorités des Tâches
Les priorités sont attribuées en fonction de la criticité de chaque tâche :
1.  **`vControlTask` (Priorité 4 - Haute) :** Le contrôle des moteurs doit être le plus réactif possible pour garantir la stabilité et l'asservissement.
2.  **`vSafetyTask` (Priorité 3) :** La détection de chute est une fonction de sécurité critique qui doit pouvoir interrompre le comportement normal du robot.
3.  **`vLidarTask` (Priorité 2) :** Le traitement des données du Lidar est important pour la navigation, mais moins critique que la sécurité immédiate.
4.  **`vImuTask` (Priorité 1 - Basse) :** La détection de choc est un événement moins fréquent et moins critique que les autres.

### Mécanismes de Synchronisation
-   **Mutex (Semaphores Mutex) :** Des mutex sont utilisés pour protéger les ressources partagées contre les accès concurrents.
    -   `xI2C1Mutex`: Protège le bus I2C1, utilisé par l'IMU et les capteurs TOF.
    -   `xUARTMutex`: Protège les accès à l'UART, notamment pour l'envoi de logs de débogage.

```c
// extrait de app_freertos.c
void MX_FREERTOS_Init(void) {
  /* Create Mutexes */
  xI2C1Mutex = xSemaphoreCreateMutex();
  xUARTMutex = xSemaphoreCreateMutex();
  // ...
}
```

## ⚙️ Drivers et HAL

Cette section décrit l'initialisation des périphériques et des drivers. Le fichier `main.c` est le point de départ pour l'initialisation de bas niveau.

```c
// extrait de main.c
int main(void)
{
  // ...
  // Initialisation des périphériques
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init(); // Lidar
  MX_USART3_UART_Init(); // Bluetooth
  MX_USART1_UART_Init(); // Debug
  MX_TIM2_Init();      // Encoder Moteur Droit
  MX_TIM3_Init();      // PWM Moteurs
  MX_TIM4_Init();      // Encoder Moteur Gauche

  // Initialisation des drivers spécifiques
  TOF_Init_All();
  // ...
  MX_FREERTOS_Init();
  vTaskStartScheduler();
  // ...
}
```

## 🎯 Stratégie comportementale

Le fichier `strategy.c` implémente la logique de haut niveau du robot. Il utilise une machine à états pour décider du comportement à adopter en fonction du rôle (Chat ou Souris) et des informations des capteurs.

### Rôles et États

-   **Rôles :** `ROLE_CHAT` (attaquer) ou `ROLE_SOURIS` (fuir). Le rôle change lors d'un choc (`Strategy_ToggleRole`).
-   **États :**
    -   `STATE_SEARCH`: Le robot tourne sur lui-même à la recherche d'une cible.
    -   `STATE_ATTACK`: Le robot (en mode Chat) se dirige vers la cible.
    -   `STATE_FLEE`: Le robot (en mode Souris) fuit la cible.
    -   `STATE_IDLE`: Le robot est à l'arrêt.

### Logique de la `Strategy_Update`

Cette fonction est appelée périodiquement par la `vControlTask`.

```c
// Extrait de strategy.c
void Strategy_Update(void) {
    LidarTarget_t target = ydlidar_get_target();

    switch (current_state) {
        case STATE_SEARCH:
            // Le robot tourne sur lui-même
            target_speed_lin_x = 0.0f;
            target_speed_ang_z = SEARCH_ROT_SPEED;

            // Si une cible est trouvée, changer d'état
            if (target.is_valid) {
                current_state = (current_role == ROLE_CHAT) ? STATE_ATTACK : STATE_FLEE;
            }
            break;
        // ...
    }
}
```

### Détail des États de la Stratégie

-   #### `STATE_IDLE`
    -   **Objectif :** Mettre le robot en état de pause.
    -   **Actions :**
        -   `target_speed_lin_x` = 0.0f
        -   `target_speed_ang_z` = 0.0f
    -   **Entrée :** Cet état est généralement activé lorsque le robot est désactivé via la commande `Strategy_SetEnabled(0)`.
    -   **Sortie :** Repasse à `STATE_SEARCH` lorsque le robot est réactivé (`Strategy_SetEnabled(1)`).

-   #### `STATE_SEARCH`
    -   **Objectif :** Trouver un autre robot dans l'environnement.
    -   **Actions :**
        -   Le robot effectue une rotation lente sur lui-même (`target_speed_ang_z = SEARCH_ROT_SPEED`).
        -   La vitesse linéaire est nulle (`target_speed_lin_x = 0.0f`).
    -   **Entrée :** C'est l'état par défaut au démarrage, ou après avoir perdu une cible.
    -   **Sortie :**
        -   Si une cible est détectée par le Lidar (`target.is_valid`):
            -   Si le rôle est `ROLE_CHAT`, passe à `STATE_ATTACK`.
            -   Si le rôle est `ROLE_SOURIS`, passe à `STATE_FLEE`.

-   #### `STATE_ATTACK`
    -   **Objectif :** (Pour le Chat) Poursuivre et intercepter la Souris.
    -   **Actions :**
        -   Un contrôleur proportionnel simple est utilisé pour s'aligner avec la cible. La vitesse de rotation (`target_speed_ang_z`) est proportionnelle à l'erreur d'angle par rapport à la cible.
        -   Si le robot est bien aligné, il avance vers la cible à une vitesse quasi-constante (`ATTACK_LIN_SPEED`).
        -   La vitesse ralentit à proximité de la cible pour un contact en douceur.
    -   **Entrée :** Depuis `STATE_SEARCH`, lorsque le robot est un `ROLE_CHAT` et qu'une cible est détectée.
    -   **Sortie :**
        -   Si la cible est perdue (`!target.is_valid`), retourne à `STATE_SEARCH`.
        -   Si le rôle change en `ROLE_SOURIS` (après un choc), la sécurité dans le code force un retour à `STATE_SEARCH`.

-   #### `STATE_FLEE`
    -   **Objectif :** (Pour la Souris) Échapper au Chat.
    -   **Actions :**
        -   La logique est l'inverse de l'attaque : le robot calcule un angle opposé (180°) à la position du Chat.
        -   Il s'oriente dans cette direction de fuite.
        -   Une fois orienté, il accélère (`FLEE_LIN_SPEED`) pour s'éloigner le plus rapidement possible.
    -   **Entrée :** Depuis `STATE_SEARCH`, lorsque le robot est une `ROLE_SOURIS` et qu'une menace est détectée.
    -   **Sortie :**
        -   Si la menace disparaît (`!target.is_valid`), retourne à `STATE_SEARCH`.
        -   Si le rôle change en `ROLE_CHAT`, retourne à `STATE_SEARCH`.

## 📊 Tests et validation
### Communication USART via Module Bluetooth HC-05

La communication sans fil avec le module Bluetooth HC-05 est essentielle pour le débogage en temps réel et la télémétrie.

-   **Utilisation :** Le module HC-05 est connecté à l'UART3 du microcontrôleur. Il est configuré pour un débit de 9600 bauds (ou autre, selon la configuration du module).
-   **Données Transmises :**
    -   **Messages de Débogage (`printf`) :** Toutes les sorties `printf` du système sont redirigées vers l'UART1 (pour la console série filaire) et vers l'UART3 (pour le Bluetooth). Cela permet de suivre le comportement du robot sans fil.
    -   **Télémétrie :** Des données critiques comme l'état actuel du robot, les distances des capteurs, les vitesses des moteurs, ou les coordonnées d'odométrie peuvent être envoyées via Bluetooth à une application externe (ex: un terminal série sur PC ou un smartphone).
-   **Vérification :**
    1.  **Connexion :** Assurez-vous que le module HC-05 est correctement appairé à votre terminal Bluetooth (PC, smartphone). Le LED du module devrait passer de clignotement rapide à lent/fixe une fois connecté.
    2.  **Test `printf` :** Après le démarrage du robot, les messages "Starting System..." et "TOF Init Done." devraient apparaître sur le terminal Bluetooth.
    3.  **Surveillance en Temps Réel :** Des messages indiquant les changements d'état de la stratégie (ATTACK, FLEE), les détections de chocs, ou les alertes de sécurité (VOID DETECTED) devraient être visibles en temps réel.
    4.  **Envoi de Commandes (Optionnel) :** Si l'application intègre la réception de commandes via Bluetooth, testez l'envoi de commandes simples (ex: "START", "STOP", "CHANGE_ROLE") depuis votre terminal pour vérifier la réactivité du robot.

```c
// Extrait de main.c pour la redirection de printf
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY); // Debug filaire
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY); // Bluetooth
	return ch;
}

// Extrait de app_freertos.c pour l'initialisation du HC-05
void MX_FREERTOS_Init(void) {
  // ...
  HC05_Init(); // Initialisation du driver Bluetooth
  // ...
}
```
<img width="1220" height="2712" alt="image" src="https://github.com/user-attachments/assets/a8628635-f0d2-4451-8770-b7ae8a8bb45b" />


## 🔧 Résultats et perspectives

Le projet illustre la mise en œuvre complète d’un système embarqué autonome, combinant conception électronique, programmation temps réel et stratégie comportementale. À travers le développement d’une carte électronique dédiée autour du STM32G431CBU6, l’intégration de capteurs variés et l’utilisation de FreeRTOS, le robot est capable d’interagir de manière fiable et réactive avec son environnement.

L’architecture logicielle modulaire, basée sur des tâches concurrentes hiérarchisées, garantit la sécurité du robot tout en assurant un contrôle précis des moteurs et une prise de décision fluide. La stratégie Chat / Souris, implémentée sous forme de machine à états, démontre la capacité du système à adapter dynamiquement son comportement en fonction des événements détectés (obstacles, chocs, présence d’un autre robot).

Les tests réalisés, notamment via la communication Bluetooth et les démonstrations en conditions réelles, valident la robustesse du système et la cohérence entre les choix matériels et logiciels. Le robot fonctionne de manière autonome, évite les chutes, détecte son environnement et interagit avec d’autres robots conformément aux objectifs du projet.

Ce projet constitue une base solide pour des améliorations futures, telles que l’intégration d’algorithmes de navigation plus avancés, l’optimisation énergétique ou l’extension vers des scénarios multi-robots plus complexes.


https://github.com/user-attachments/assets/d23f3977-9c8e-4610-84ae-4908d1821f06


