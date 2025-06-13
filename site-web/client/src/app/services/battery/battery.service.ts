import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { Subject } from 'rxjs';

@Injectable({
    providedIn: 'root',
})
export class BatteryService {
    private batteryLevelSubject = new Subject<number>();
    batteryLevel$ = this.batteryLevelSubject.asObservable();
    private socket: Socket | null = null;

    constructor() {
        this.socket = io('http://localhost:3000'); 
        this.listenForBatteryUpdates();
    }

    private listenForBatteryUpdates() {
        if (!this.socket) return;

        this.socket.on('battery_update', (data: { battery: number }) => {
            console.log('📩 Mise à jour batterie reçue :', data);
            this.batteryLevelSubject.next(data.battery);
        });
    }

    requestBatteryStatus(ip: string) {
        if (!this.socket) return;
        console.log(`📡 Demande de niveau de batterie pour ${ip}`);
        this.socket.emit('get_battery', { ip });
    }
}
