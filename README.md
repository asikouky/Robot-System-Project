# Projet INF3995 – Système Robotique d'Exploration Hiver 2025 (Équipe - 104)

## Description du Projet 🚀

Ce projet vise à concevoir un système complet permettant à une équipe de deux robots d’explorer un environnement intérieur, en collectant des données via caméra et capteur laser, et en les restituant en temps réel à une station de contrôle accessible via une interface web.

Le système est composé de trois parties principales :
- **Station au sol :** Interface utilisateur web, gestion de la base de données et des communications avec les robots.
- **Robots physiques :** Équipement embarqué avec ROS (Python), capteurs et algorithmes d'exploration autonome.
- **Simulation Gazebo :** Simulation avancée pour valider les comportements des robots avant déploiement physique.

---

## Architecture du Projet 🧩

### 🖥️ Station au sol (Frontend / Backend / DB)
- **Frontend :** Angular
- **Backend :** Node.js (Express.js)
- **Base de données :** MongoDB
- **Communication :** WebSocket / HTTP REST
- **Déploiement :** Docker-compose

### 🤖 Robots physiques
- **Robot OS :** ROS2 (Python)
- **Contrôle des moteurs :** Algorithmes d'exploration autonome
- **Capteurs :** Caméra, LiDAR
- **Communication :** P2P entre robots, WebSocket avec station au sol
- **Gestion de la batterie :** Monitoring en temps réel

### 🧭 Simulation Gazebo
- Simulation 3D de l’environnement avec obstacles générés aléatoirement
- Test des algorithmes de navigation et d'évitement d'obstacles
- Interface identique à celle des robots physiques

---

## Installation 🛠️

### Prérequis
- Docker / Docker Compose
- Node.js / npm
- Python
- ROS2 (Humble)
- MongoDB

### Lancement rapide

#### 1. Clonez le dépôt :
   ```bash
   git clone https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104.git
   ```

#### 2. Lancer la station au sol - siteweb (frontend + backend + DB) :

Accédez au dossier du [site web](https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104/-/tree/main/site-web?ref_type=heads)

###### Pour lancer le client Angular :
```bash
cd site-web
npm ci       # Installer les dépendances
npm start    # Lancer le client
```

###### Pour lancer le serveur Node.js :
```bash
cd site-web
npm ci       # Installer les dépendances
npm start    # Lancer le serveur
```

###### Pour exécuter les tests côté client et serveur :
```bash
npm run test
```
#### 3. Lancer la simulation Gazebo :
   ```bash
   docker-compose
   ```

#### 4. Lancer les robots physiques :
 ```bash
   ./build_and_launch.sh
   ```
N.B : pour chaque requis, veuillez vous referer au document [Tests.pdf](https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104)

---

### Pour lancer le serveur et client du serveur
Aller dans le fichier site-web et puit faite
```bash
docker compose up --build
```

### Pour lancer la simulation d'un seul coup
Aller dans le fichier robot et lancer la commande suivante.
```bash
./prod_launch.sh
```
si vous voulez utiliser le mode ackerman
```bash
./prod_launch.sh --enable_ackerman
```
Utilisez VNC et faites une connection à localhost:5901

###  Démonstration Vidéos

Les vidéos de démonstration sont disponibles dans le dossier :  ➡️ [demos](https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104/-/tree/main/demos?ref_type=heads)



---


## Équipe 👥

- **Abdelnour Sikouky**
- **Zineb Benzeroual**
- **Thomas Spina**
- **Junior Stevy Randy Boussougou**
- **Thomas Spina**
- **Mariem ben Jaber**
- **Ryan Kezouh**



---

## Documentation 📚

- [Documentation technique Finale](https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104)
- [Procédures de test](https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104/-/blob/main/Tests-Equipe104-RR.pdf?ref_type=heads)
- [Vidéos de démonstration](https://gitlab.com/polytechnique-montr-al/inf3995/20251/equipe-104/INF3995-104/-/tree/main/demos?ref_type=heads)

---

## License 📄

Projet réalisé dans le cadre du cours **INF3995 - Polytechnique Montréal**  
© 2025 - Tous droits réservés
