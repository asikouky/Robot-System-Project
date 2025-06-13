#!/bin/bash

# Modifier environment.ts pour utiliser localhost
envFile="src/environments/environment.ts"

echo "Mise à jour de $envFile pour utiliser localhost"

cat <<EOL > $envFile
export const environment = {
  production: false,
  serverUrl: 'http://localhost:3000'
};
EOL

echo "✅ environment.ts mis à jour avec : http://localhost:3000"

# Lancer Angular
echo "🚀 Lancement du client Angular..."
npm run start
