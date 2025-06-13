import { Component, OnInit, ViewChild, ElementRef, AfterViewInit } from '@angular/core';
import { WebSocketService } from '@app/services/socket/websocket.service';

interface OccupancyGridInfo {
  width: number;
  height: number;
  resolution: number;
  origin: {
    position: { x: number, y: number, z: number },
    orientation: { x: number, y: number, z: number, w: number }
  };
}

interface MapData {
  info: OccupancyGridInfo;
  data: number[];
}

interface Pose {
  position: { 
    x: number;
    y: number;
    z: number;
  };
  orientation: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
}

@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
  standalone: true
})
export class MapComponent implements OnInit, AfterViewInit {
  @ViewChild('mapCanvas') canvas: ElementRef<HTMLCanvasElement>;
  private ctx: CanvasRenderingContext2D;
  currentMap: MapData;
  currentPose: Pose; 

  constructor(private webSocketService: WebSocketService) {}

  ngOnInit(): void {
   
    this.webSocketService.connect('ws://votre_ip:8765').subscribe({
      next: (message: MessageEvent) => this.handleMessage(message),
      error: (err) => console.error('Erreur WebSocket:', err),
      complete: () => console.warn('Connexion WebSocket fermée')
    });
  }

  ngAfterViewInit(): void {
    const context = this.canvas.nativeElement.getContext('2d');
    if (context) {
      this.ctx = context;
    } else {
      console.error('Impossible d’obtenir le contexte 2D du canvas');
    }
  }

  handleMessage(message: MessageEvent): void {
    const data = JSON.parse(message.data);
    console.log('[WebSocket] Données reçues :', data);

   
    if (data.map) {
      this.currentMap = data.map;
      console.log('[Map] OccupancyGrid reçu :', this.currentMap);
      this.drawMap();
    }

    
    if (data.pose) {
      this.currentPose = data.pose;
      console.log('[Pose] Pose reçue :', this.currentPose);
      
      this.drawMap();
    }
  }

  drawMap(): void {
    if (!this.ctx || !this.currentMap) return;

    const { width, height } = this.currentMap.info;
    const scale = 20; 
    const canvasEl = this.canvas.nativeElement;

   
    canvasEl.width = width * scale;
    canvasEl.height = height * scale;

   
    this.ctx.clearRect(0, 0, canvasEl.width, canvasEl.height);

    
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const idx = y * width + x;
        const cell = this.currentMap.data[idx];
        let color: string;

        if (cell === 100) {
          color = 'black';   // Zone occupée
        } else if (cell === 0) {
          color = 'green';   // Zone libre (vous pouvez modifier par 'white' selon vos préférences)
        } else if (cell === -1) {
          color = 'gray';    // Zone inconnue
        } else {
        
          let intensity = 255 - Math.floor(cell * 255 / 100);
          color = `rgb(${intensity}, ${intensity}, ${intensity})`;
        }

        this.ctx.fillStyle = color;
        this.ctx.fillRect(x * scale, y * scale, scale, scale);
      }
    }

    
    if (this.currentPose) {
      this.drawRobotMarker(scale);
    }
  }

  drawRobotMarker(scale: number): void {
    if (!this.ctx || !this.currentPose || !this.currentMap) return;
  
    const originX = this.currentMap.info.origin.position.x;
    const originY = this.currentMap.info.origin.position.y;
    const xPixel = (this.currentPose.position.x - originX) * scale;
    const yPixel = (this.currentPose.position.y - originY) * scale;
    
  
    
    const radius = 10;
    
    this.ctx.beginPath();
    this.ctx.arc(xPixel, yPixel, radius, 0, 2 * Math.PI);
    this.ctx.fillStyle = 'red';
    this.ctx.fill();
    
  
    this.ctx.lineWidth = 10;
    this.ctx.strokeStyle = 'black';
    this.ctx.stroke();
  }
  
  
  
}
