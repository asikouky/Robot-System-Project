import { Component, OnInit } from '@angular/core';
import { BatteryService } from '../../services/battery/battery.service';
import { CommonModule } from '@angular/common';

@Component({
    selector: 'app-battery-status',
    standalone: true,
    imports: [CommonModule],
    templateUrl: './battery-status.component.html',
    styleUrl: './battery-status.component.scss',
})
export class BatteryStatusComponent implements OnInit {
    batteryLevel: number = 100; 

    constructor(private batteryService: BatteryService) {}

    ngOnInit() {
        this.batteryService.batteryLevel$.subscribe((level) => {
            this.batteryLevel = level;
        });
    }

    getBatteryColor(): string {
        if (this.batteryLevel > 50) return 'green';
        if (this.batteryLevel > 30) return 'orange';
        return 'red';
    }
}
