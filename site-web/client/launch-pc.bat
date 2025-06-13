@echo off

:: Modifier environment.ts pour utiliser localhost
echo Mise a jour de src\environments\environment.ts pour utiliser localhost...

echo export const environment = { > src\environments\environment.ts
echo   production: false, >> src\environments\environment.ts
echo   serverUrl: 'http://localhost:3000' >> src\environments\environment.ts
echo }; >> src\environments\environment.ts

echo environment.ts mis a jour avec : http://localhost:3000

:: Lancer Angular
echo Lancement du client Angular...
npm run start
