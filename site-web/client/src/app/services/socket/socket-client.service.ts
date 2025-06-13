// from https://gitlab.com/nikolayradoev/socket-io-exemple/-/blob/master/client/src/app/services/socket-client.service.ts?ref_type=heads
/* eslint-disable */

import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { environment } from 'src/environments/environment';

@Injectable({
    providedIn: 'root',
})
export class SocketClientService {
    socket: Socket;

    isSocketAlive() {
        return this.socket && this.socket.connected;
    }

    connect() {
        this.socket = io(environment.serverUrl, { transports: ['websocket'], upgrade: false });

        this.socket.on('connect', () => {
            console.log(`[WebSocket] Connecté au serveur avec ID : ${this.socket.id}`);
        });

        this.socket.on('disconnect', (reason) => {
            console.log(`[WebSocket] Déconnecté : ${reason}`);
        });

        this.socket.on('connect_error', (err) => {
            console.error(`[WebSocket] Erreur de connexion : ${err.message}`);
        });
    }

    disconnect() {
        this.socket.disconnect();
    }

    on<T>(event: string, action: (data: T) => void): void {
        this.socket.on(event, action);
    }

    // eslint-disable-next-line @typescript-eslint/ban-types
    send<T>(event: string, data?: T, callback?: Function): void {
        this.socket.emit(event, ...[data, callback].filter((x) => x));
    }
}
