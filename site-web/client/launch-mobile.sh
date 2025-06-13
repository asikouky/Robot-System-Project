#!/bin/bash

# Demander l'adresse IP
echo "Entrez l'adresse IP du serveur (PC) pour la connexion mobile : "
read ip

# Pattern de validation d'adresse IP
ipPattern="^(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)$"

# Vérifier si l'IP est valide
if [[ ! $ip =~ $ipPattern ]]; then
    echo "❌ Adresse IP invalide. Veuillez entrer une adresse au format X.X.X.X"
    exit 1
fi

# Modifier l'`environment.ts`
envFile="src/environments/environment.ts"

echo "Mise à jour de $envFile avec l'adresse IP : $ip"

cat <<EOL > $envFile
export const environment = {
  production: false,
  serverUrl: 'http://$ip:3000'
};
EOL

echo "✅ environment.ts mis à jour avec : http://$ip:3000"

# Afficher l'URL à ouvrir sur mobile
echo ""
echo "📱 Ouvrez sur votre navigateur mobile et allez à :"
echo "   👉 http://$ip:4200"
echo ""

# Lancer Angular avec la nouvelle configuration
ng serve --host 0.0.0.0 --port 4200
