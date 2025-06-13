import { Injectable } from '@angular/core';
import { Observable, Observer } from 'rxjs';
import { environment } from 'src/environments/environment';

@Injectable({
  providedIn: 'root'
})
export class WebSocketService {
  private socket: WebSocket;

  // Se connecter en utilisant l'URL définie dans l'environnement
  // Dans web-socket.service.ts
connect(url?: string): Observable<MessageEvent> {
  const connectionUrl = environment.websocketUrl;
  this.socket = new WebSocket(connectionUrl);
  return new Observable((observer: Observer<MessageEvent>) => {
    this.socket.onmessage = (event: MessageEvent) => observer.next(event);
    this.socket.onerror = (error) => observer.error(error);
    this.socket.onclose = () => observer.complete();
    return () => this.socket.close();
  });
}


  send(data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify(data));
    }
  }
}
