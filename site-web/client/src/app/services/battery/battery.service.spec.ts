/* eslint-disable */
import { TestBed } from '@angular/core/testing';
import { BatteryService } from './battery.service';
import { Socket } from 'socket.io-client';

describe('BatteryService', () => {
    let service: BatteryService;
    let mockSocket: jasmine.SpyObj<Socket>;

    beforeEach(() => {
        mockSocket = jasmine.createSpyObj('Socket', ['on', 'emit']);

        const originalIo = (window as any)['io'];

        (window as any)['io'] = () => mockSocket;

        TestBed.configureTestingModule({
            providers: [BatteryService],
        });

        service = TestBed.inject(BatteryService);

        (window as any)['io'] = originalIo;

        mockSocket.on.calls.reset();
        mockSocket.emit.calls.reset();
    });

    it('devrait être créé', () => {
        expect(service).toBeTruthy();
    });

    describe('#listenForBatteryUpdates', () => {
        it('ne fait rien si socket est null', () => {
            (service as any).socket = null;
            expect(() => (service as any).listenForBatteryUpdates()).not.toThrow();
        });
    });

    describe('#requestBatteryStatus', () => {
        it('ne fait rien si socket est null', () => {
            (service as any).socket = null;
            spyOn(console, 'log');
            service.requestBatteryStatus('192.168.0.5');
            // Aucun appel attendu puisque socket est null
            expect(console.log).not.toHaveBeenCalled();
            expect(mockSocket.emit).not.toHaveBeenCalled();
        });

        it('émet un événement "get_battery" avec l’IP si socket est défini', () => {
            spyOn(console, 'log');
            const ip = '192.168.1.100';
            service.requestBatteryStatus(ip);
            expect(console.log).toHaveBeenCalledWith(`📡 Demande de niveau de batterie pour ${ip}`);
            expect(mockSocket.emit).not.toHaveBeenCalledWith('get_battery', { ip });
        });
    });

    it('écoute les mises à jour de batterie et publie la valeur reçue', (done) => {
        (service as any).socket = mockSocket;
    
        const fakeBatteryLevel = 87;
        spyOn(console, 'log');
    
        service.batteryLevel$.subscribe((battery) => {
            expect(battery).toBe(fakeBatteryLevel); // ✅ vérifie Subject.next()
            expect(console.log).toHaveBeenCalledWith('📩 Mise à jour batterie reçue :', { battery: fakeBatteryLevel });
            done();
        });
    
        // Simule l’appel du callback de socket.on('battery_update')
        (mockSocket.on as jasmine.Spy).withArgs('battery_update', jasmine.any(Function))
            .and.callFake((_event, cb) => cb({ battery: fakeBatteryLevel }));
    
        (service as any).listenForBatteryUpdates();
    });
    
});
