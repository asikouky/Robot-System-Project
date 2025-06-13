@echo off
setlocal

:: === Demande l'IP ===
set /p ip="Entrez l'adresse IP du serveur (PC) pour la connexion mobile : "

:: === Vérifie l'IP ===
for /f "tokens=1-4 delims=." %%a in ("%ip%") do (
    set /a "ok=1"
    for %%n in (%%a %%b %%c %%d) do (
        if %%n LSS 0 set /a "ok=0"
        if %%n GTR 255 set /a "ok=0"
    )
)
if "%ok%" NEQ "1" (
    echo Adresse IP invalide. Veuillez entrer une adresse correcte.
    exit /b
)

:: === Met à jour environment.ts ===
echo export const environment = { > src\environments\environment.ts
echo   production: false, >> src\environments\environment.ts
echo   serverUrl: 'http://%ip%:3000' >> src\environments\environment.ts
echo }; >> src\environments\environment.ts

echo.
echo ✅ environment.ts mis à jour avec : http://%ip%:3000
echo 🌐 Lancez votre navigateur et accédez à : http://%ip%:4200
echo.

:: === Lancer ng serve dans une nouvelle fenêtre PowerShell ===
start "ng-serve" powershell -NoExit -Command "ng serve --host 0.0.0.0 --port 4200"

:: === Attendre que la fenêtre soit fermée ===
echo 🔄 En attente de la fermeture de ng serve...

:waitLoop
timeout /t 5 >nul
tasklist /FI "WINDOWTITLE eq ng-serve" | find /I "powershell.exe" >nul
if not errorlevel 1 goto waitLoop

:: === Réinitialise l'IP vers localhost ===
echo export const environment = { > src\environments\environment.ts
echo   production: false, >> src\environments\environment.ts
echo   serverUrl: 'http://localhost:3000' >> src\environments\environment.ts
echo }; >> src\environments\environment.ts

echo.
echo ✅ environment.ts réinitialisé avec localhost.
pause
