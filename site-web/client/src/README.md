# Guide de Lancement du Client Angular

Ce projet Angular peut être lancé :
- 💻 **Sur PC** (Windows, Linux, Mac) avec `localhost`
- 📱 **Sur un Mobile/Tablette** en utilisant l'IP du PC sur le réseau local

---

## 🔹 **Prérequis**
Avant de lancer le client, assurez-vous d'avoir :
- **Node.js** installé ([Télécharger ici](https://nodejs.org/))
- **Angular CLI** installé :
  ```sh
  npm install -g @angular/cli
  ```
- **Les dépendances du projet installées :** 
  ```sh
  npm ci
  ```
**Le serveur Node.js/WebSocket en cours d'exécution** (`npm run start` dans le dossier `server`).

---

## 💻 **Lancer le Client sur PC**
> **Windows & Linux/Mac**

📺 Cette commande mettra `serverUrl` sur `http://localhost:3000` et démarrera le client.

### **Windows**
1. **Double-cliquez** sur `launch-local.bat`
2. **Ou exécutez la commande suivante** dans un terminal :
   ```sh
   launch-local.bat
   ```

### **Linux / Mac**
1. **Donnez les permissions d’exécution au script (une seule fois)** :
   ```sh
   chmod +x launch-local.sh
   ```
2. **Lancez le script** :
   ```sh
   ./launch-local.sh
   ```

📍 **Accès au site sur PC** :
```
http://localhost:4200
```

---

## 📱 **Lancer le Client pour un Accès Mobile**
> **Windows & Linux/Mac**

🛠️ Cette commande **demande une adresse IP**, met à jour `serverUrl`, et lance Angular.

### **Windows**
1. **Double-cliquez** sur `launch-mobile.bat`
2. **Ou exécutez la commande suivante** dans un terminal :
   ```sh
   launch-mobile.bat
   ```

### **Linux / Mac**
1. **Donnez les permissions d’exécution au script (une seule fois)** :
   ```sh
   chmod +x launch-mobile.sh
   ```
2. **Lancez le script** :
   ```sh
   ./launch-mobile.sh
   ```

🛠️ **Lorsque le script vous demande une adresse IP** :  
- Tapez **l'IP de votre PC sur le réseau** (ex : `192.168.X.X` ou `10.200.X.X`).
- **Si vous ne connaissez pas votre IP** :
  - **Windows** : Ouvrez `cmd` et tapez (c'est souvent l'adresse IPv4 dans Carte réseau sans fil Wi-fi) :
    ```sh
    ipconfig
    ```
  - **Linux/Mac** : Ouvrez un terminal et tapez :
    ```sh
    ifconfig | grep "inet " | grep -v 127.0.0.1
    ```

📍 **Accès au site sur Mobile/Tablette** :  
> Une fois le script lancé, ouvrez un navigateur mobile et entrez :
```
http://<votre IP>:4200
```
(Par exemple, `http://10.200.56.180:4200`)

---

## 🛠️ **Dépannage**
### 1️⃣ **Problème de connexion depuis le téléphone**
- Assurez-vous que **le PC et le téléphone sont sur le même réseau Wi-Fi**.
- **Désactivez temporairement votre pare-feu** sous Windows :
  ```sh
  netsh advfirewall set allprofiles state off
  ```
  (*Ne pas oublier de le réactiver après !*)

### 2️⃣ **Erreur "ng serve: command not found"**
- Assurez-vous que **Angular CLI est installé** :
  ```sh
  npm install -g @angular/cli
  ```

### 3️⃣ **Problème d’accès au serveur WebSocket**
- Vérifiez que votre **serveur Node.js tourne bien** en lançant :
  ```sh
  npm run start
  ```
  dans le dossier `server/`.

---

## 📚 **Résumé des Commandes**
| Action | Windows | Linux / Mac |
|--------|---------|-------------|
| **Lancer le client sur PC** | `launch-local.bat` | `./launch-local.sh` |
| **Lancer le client sur mobile** | `launch-mobile.bat` | `./launch-mobile.sh` |

---

## 🚀 **Prêt à tester !**
Vous pouvez maintenant lancer le projet et accéder à votre site depuis **votre PC ou votre téléphone** 🎉.

