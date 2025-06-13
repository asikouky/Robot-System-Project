import { CommonModule } from '@angular/common';
import { AfterViewChecked, ChangeDetectorRef, Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { MatDialogModule } from '@angular/material/dialog';
import { ActivatedRoute } from '@angular/router';
import { BatteryStatusComponent } from '@app/components/battery-status/battery-status.component';
import { DialogAddRobotComponent } from '@app/components/dialog-add-robot/dialog-add-robot.component';
import { MapComponent } from '@app/components/map/map/map.component';
import { MenuBarComponent } from '@app/components/menu-bar/menu-bar.component';
import { AppMaterialModule } from '@app/modules/material.module';
import { SocketClientService } from '@app/services/socket/socket-client.service';
import { MissionStatus } from '@common/action.robot';
import { ws_Methods } from '@common/events';

@Component({
    selector: 'app-mission-page',
    standalone: true,
    imports: [CommonModule, MenuBarComponent, AppMaterialModule, MatDialogModule, BatteryStatusComponent, DialogAddRobotComponent, MapComponent],
    templateUrl: './mission-page.component.html',
    styleUrls: ['./mission-page.component.scss'],
})
export class MissionPageComponent implements OnInit, AfterViewChecked {
    disableLaunchMission: boolean = true;
    isDialogOpen: boolean = false;
    firstRobotIP: string = '';
    secondRobotIP: string = '';
    logMessages: string[] = [];
    autoScroll: boolean = true;
    logsVisible: boolean = false;
    timeLeft: number = 0;
    MissionStatus = MissionStatus;
    currentMissionStatus: MissionStatus = MissionStatus.IDLE;
    mapImageBase64: string | null = null;
    selectedView: string = 'simulation';

    @ViewChild('logsContainer') private logsContainer?: ElementRef;

    constructor(
        private socketService: SocketClientService,
        private route: ActivatedRoute,
        private cdRef: ChangeDetectorRef,
    ) {}

    ngOnInit(): void {
        this.currentMissionStatus = MissionStatus.IDLE;
        this.cdRef.detectChanges();

        this.route.paramMap.subscribe((params) => {
            this.firstRobotIP = params.get('ip') || '';
            console.log('🔍 IP du robot reçue:', this.firstRobotIP);
        });

        this.connect();
        this.setupSocketListeners();
    }

    ngAfterViewChecked(): void {
        this.scrollToBottom();
    }

    connect(): void {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect();
        }
    }

    setupSocketListeners(): void {
        this.socketService.socket.on(ws_Methods.MISSION_STATUS_UPDATE, (data) => {
            this.currentMissionStatus = data.status;
            this.cdRef.detectChanges();
        });

        this.socketService.socket.on(ws_Methods.MAP_IMAGE_UPDATE, (data: { mapImage: string }) => {
            this.mapImageBase64 = data.mapImage;
            console.log("map recu", this.mapImageBase64);
          });

        this.socketService.socket.on(ws_Methods.TIMER_UPDATE, (data: { time: number }) => {
            this.timeLeft = data.time;
        });

        this.socketService.on(ws_Methods.LASER_SCAN_LOG, (logString: string) => {
            this.addLogMessage(logString);
        });

        this.socketService.on(ws_Methods.ERROR, (errorMsg: string) => {
            this.addLogMessage(`ERROR: ${errorMsg}`);
        });

        this.socketService.on(ws_Methods.STATUS, (statusMsg: string) => {
            this.addLogMessage(`STATUS: ${statusMsg}`);
        });
    }

    private scrollToBottom(): void {
        if (this.autoScroll && this.logsContainer && this.logsVisible) {
            const element = this.logsContainer.nativeElement;
            element.scrollTop = element.scrollHeight;
        }
    }

    toggleAutoScroll(): void {
        this.autoScroll = !this.autoScroll;
        if (this.autoScroll) {
            this.scrollToBottom();
        }
    }

    formatTime(seconds: number): string {
        const minutes = Math.floor(seconds / 60);
        const remainingSeconds = seconds % 60;
        return `${minutes}m ${remainingSeconds}s`;
    }

    getMissionStatusClass(): string {
        switch (this.currentMissionStatus) {
            case MissionStatus.IDLE:
                return 'status-idle';
            case MissionStatus.IN_MISSION:
                return 'status-active';
            case MissionStatus.RETURNING:
                return 'status-returning';
            case MissionStatus.UPDATING:
                return 'status-updating';
            case MissionStatus.DISCONNECTED:
                return 'status-disconnected';
            default:
                return 'status-idle';
        }
    }

    finish(): void {
        this.socketService.send(ws_Methods.STOP_MISSION, { ip: this.firstRobotIP });
    }

    toggleLogsVisibility(): void {
        this.logsVisible = !this.logsVisible;
        if (this.logsVisible && this.autoScroll) {
            setTimeout(() => this.scrollToBottom(), 100);
        }
    }

    addLogMessage(message: string): void {
        const now = new Date();
        const timestamp = `[${now.toLocaleTimeString()}]`;
        this.logMessages.push(`${timestamp} ${message}`);
    }

    clearLogs(): void {
        this.logMessages = [];
    }
    selectView(view: string): void {
        this.selectedView = view;
    }
}
