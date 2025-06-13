/* eslint-disable */
import { ComponentFixture, TestBed } from '@angular/core/testing';
import { BatteryStatusComponent } from './battery-status.component';
import { BehaviorSubject } from 'rxjs';
import { BatteryService } from '../../services/battery/battery.service';
import { CommonModule } from '@angular/common';

describe('BatteryStatusComponent', () => {
    let component: BatteryStatusComponent;
    let fixture: ComponentFixture<BatteryStatusComponent>;
    let batteryLevelSubject: BehaviorSubject<number>;
    let batteryServiceStub: Partial<BatteryService>;

    beforeEach(async () => {
        batteryLevelSubject = new BehaviorSubject<number>(100); // Valeur par défaut
        batteryServiceStub = {
            batteryLevel$: batteryLevelSubject.asObservable(),
        };

        await TestBed.configureTestingModule({
            imports: [BatteryStatusComponent, CommonModule],
            providers: [{ provide: BatteryService, useValue: batteryServiceStub }],
        }).compileComponents();

        fixture = TestBed.createComponent(BatteryStatusComponent);
        component = fixture.componentInstance;
        fixture.detectChanges(); // Appelle ngOnInit()
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should initialize with default battery level', () => {
        expect(component.batteryLevel).toBe(100);
    });

    it('should update batteryLevel when observable emits new value', () => {
        batteryLevelSubject.next(45);
        fixture.detectChanges();
        expect(component.batteryLevel).toBe(45);
    });

    describe('getBatteryColor', () => {
        it('should return green when batteryLevel > 50', () => {
            component.batteryLevel = 75;
            expect(component.getBatteryColor()).toBe('green');
        });

        it('should return orange when batteryLevel > 30 and <= 50', () => {
            component.batteryLevel = 40;
            expect(component.getBatteryColor()).toBe('orange');
        });

        it('should return red when batteryLevel <= 30', () => {
            component.batteryLevel = 20;
            expect(component.getBatteryColor()).toBe('red');
        });
    });
});
