import { Component, OnInit } from '@angular/core';
import { FormControl, FormGroup, FormsModule, ReactiveFormsModule, Validators } from '@angular/forms';
import { MatDialogModule } from '@angular/material/dialog';
import { Router } from '@angular/router';
import { AppMaterialModule } from '@app/modules/material.module';
import { RobotService } from '@app/services/robot/robot.service';
import { SocketClientService } from '@app/services/socket/socket-client.service';
import { ModeMission, ws_Methods } from '@common/events';

@Component({
    selector: 'app-dialog-add-robot',
    standalone: true,
    imports: [ReactiveFormsModule, FormsModule, AppMaterialModule, MatDialogModule],
    templateUrl: './dialog-add-robot.component.html',
    styleUrl: './dialog-add-robot.component.scss',
})
export class DialogAddRobotComponent implements OnInit {
    ModeMission = ModeMission; 
    message: string = '';
    isValidIP: boolean = false;
    isValidSecondIP: boolean = false;
    serverMessages: string[] = [];
    isLoadingActive: boolean = false;
    isMissionAvailable: boolean = true;
    robotIp: string = '';
    addSecondRobot: boolean = false;
    profileForm = new FormGroup({
        ipAddress: new FormControl<string>('', [
            Validators.required,
            Validators.pattern(
                /^(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)$/,
            ),
        ]),
        ipAddress2: new FormControl<string>(''), 
        mode: new FormControl<ModeMission | null>(null, Validators.required),
    });

    constructor(
        private socketService: SocketClientService,
        private robotService: RobotService,
        private router: Router,
    ) {}

    ngOnInit(): void {
        this.connect();
        this.listenForServerMessages();
    }

    connect() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect();
        }
    }

    listenForServerMessages(): void {
        this.socketService.on(ws_Methods.STATUS, (msg: string) => {
            this.serverMessages.push(`[STATUS] ${msg}`);
        });

        this.socketService.on(ws_Methods.ERROR, (msg: string) => {
            this.serverMessages.push(`[ERROR] ${msg}`);
        });

        this.socketService.on(ws_Methods.MISSION_AVAILABILITY, (state: boolean) => {
            this.isMissionAvailable = state;
        });
    }

    get ipAddressControl(): FormControl {
        return this.profileForm.get('ipAddress') as FormControl;
    }

    get isModeSelected(): boolean {
        return this.profileForm.get('mode')?.valid ?? false;
    }

    get secondIpAddressControl(): FormControl {
        return this.profileForm.get('ipAddress2') as FormControl;
    }

    formatIPAddress(fieldName: 'ipAddress' | 'ipAddress2'): void {
        let ip = this.profileForm.controls[fieldName].value || '';
        ip = ip.replace(/[^0-9.]/g, '');
        ip = ip.replace(/\.{2,}/g, '.');
    
        if (ip === '') {
            this.profileForm.controls[fieldName].setValue('', { emitEvent: false });
            return;
        }
    
        let parts = ip.split('.').map((part) => part.substring(0, 3));
        let formattedIP = parts.join('.');
    
        if (
            formattedIP.length < 15 &&
            /^[0-9]{3}$|^[0-9]{3}\.[0-9]{3}$|^[0-9]{3}\.[0-9]{3}\.[0-9]{3}$/.test(formattedIP)
        ) {
            formattedIP += '.';
        }
    
        formattedIP = formattedIP
            .split('.')
            .map((part) => {
                const num = parseInt(part, 10);
                return isNaN(num) ? '' : Math.min(num, 255).toString();
            })
            .join('.');
    
        const blocks = formattedIP.split('.');
        if (blocks.length > 4) {
            formattedIP = blocks.slice(0, 4).join('.');
        }
    
        this.profileForm.controls[fieldName].setValue(formattedIP, { emitEvent: false });
    }



    validateIP(event: Event): void {
        event.preventDefault();
        const ip = this.profileForm.get('ipAddress')!.value || '';
        const ip2 = this.profileForm.get('ipAddress2')?.value || '';

        const ipPattern =
            /^(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|1?[0-9][0-9]?)$/;
        this.isValidIP = ipPattern.test(ip);
        this.isValidSecondIP = this.addSecondRobot ? ipPattern.test(ip2) : false;

    }

    identify(fieldName: 'ipAddress' | 'ipAddress2'): void {
        const ip = this.profileForm.get(fieldName)?.value || '';
        const mode = this.profileForm.get('mode')?.value as ModeMission;
        const numRobot = fieldName === 'ipAddress' ? '1' : '2';
        this.socketService.send(ws_Methods.IDENTIFY, { ip: ip, numRobot: numRobot, mode: mode });      
    }
    

    launch(): void {
        const ip = this.profileForm.get('ipAddress')?.value || '';
        const ip2 = this.profileForm.get('ipAddress2')?.value || '';
        const mode = this.profileForm.get('mode')?.value as ModeMission;
        this.socketService.send(ws_Methods.LAUNCH_MISSION, { ip: ip, ip2: ip2, mode: mode });
        // a faire avec la rep du server
        this.router.navigate(['/mission', ip, ip2]);
    }

    startRos() {
        if (!this.robotIp) {
            this.message = 'Veuillez entrer une adresse IP.';
            return;
        }

        console.log(`Démarrage de ROS sur ${this.robotIp}`);
        this.robotService.startRobot(this.robotIp).subscribe({
            next: (response) => {
                console.log('Réponse du serveur :', response);
                this.message = `ROS démarré sur ${this.robotIp} !`;
            },
            error: (error) => {
                console.error('Erreur lors du démarrage de ROS :', error);
                this.message = `Erreur : Impossible de démarrer ROS sur ${this.robotIp}.`;
            },
        });
    }
    activerP2P() {
        this.socketService.send('message', { command: 'P2P' });
        console.log('✅ Commande P2P envoyée via WebSocket');
      }
    
      desactiverP2P() {
        this.socketService.send('message', { command: 'P2P_OFF' });
        console.log('❎ Commande P2P_OFF envoyée via WebSocket');
      }
      
}
