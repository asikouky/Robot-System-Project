
// import { ComponentFixture, TestBed } from '@angular/core/testing';
// import { DialogAddRobotComponent } from './dialog-add-robot.component';
// import { Router } from '@angular/router';
// import { of, throwError } from 'rxjs';
// import { SocketClientService } from '@app/services/socket/socket-client.service';
// import { RobotService } from '@app/services/robot/robot.service';
// import { ModeMission, ws_Methods } from '@common/events';
// import { FormGroup } from '@angular/forms';

// describe('DialogAddRobotComponent', () => {
//     let component: DialogAddRobotComponent;
//     let fixture: ComponentFixture<DialogAddRobotComponent>;
//     let socketServiceSpy: jasmine.SpyObj<SocketClientService>;
//     let robotServiceSpy: jasmine.SpyObj<RobotService>;
//     let routerSpy: jasmine.SpyObj<Router>;

//     beforeEach(async () => {
//         socketServiceSpy = jasmine.createSpyObj('SocketClientService', ['connect', 'isSocketAlive', 'on', 'send']);
//         robotServiceSpy = jasmine.createSpyObj('RobotService', ['startRobot']);
//         routerSpy = jasmine.createSpyObj('Router', ['navigate']);

//         await TestBed.configureTestingModule({
//             imports: [DialogAddRobotComponent],
//             providers: [
//                 { provide: SocketClientService, useValue: socketServiceSpy },
//                 { provide: RobotService, useValue: robotServiceSpy },
//                 { provide: Router, useValue: routerSpy },
//             ],
//         }).compileComponents();

// //         fixture = TestBed.createComponent(DialogAddRobotComponent);
// //         component = fixture.componentInstance;
// //         fixture.detectChanges();
// //     });

//     it('should create', () => {
//         expect(component).toBeTruthy();
//     });

//     it('should connect if socket not alive', () => {
//         socketServiceSpy.isSocketAlive.and.returnValue(false);
//         component.connect();
//         expect(socketServiceSpy.connect).toHaveBeenCalled();
//     });

//     it('should not connect if socket is alive', () => {
//         socketServiceSpy.isSocketAlive.and.returnValue(true);
//         component.connect();
//         expect(socketServiceSpy.connect).toHaveBeenCalled();
//     });

//     it('should validate valid IP addresses', () => {
//         component.profileForm.controls['ipAddress'].setValue('192.168.0.1');
//         component.validateIP(new Event('submit'));
//         expect(component.isValidIP).toBeTrue();
//     });




//     it('should set message when IP missing in startRos', () => {
//         component.robotIp = '';
//         component.startRos();
//         expect(component.message).toContain('Veuillez entrer une adresse IP');
//     });

//     it('should call robotService and set message on success', () => {
//         component.robotIp = '192.168.0.1';
//         robotServiceSpy.startRobot.and.returnValue(of('OK'));
//         component.startRos();
//         expect(component.message).toContain('ROS démarré');
//     });

//     it('should call robotService and set message on error', () => {
//         component.robotIp = '192.168.0.1';
//         robotServiceSpy.startRobot.and.returnValue(throwError(() => new Error('fail')));
//         component.startRos();
//         expect(component.message).toContain('Erreur');
//     });

//     it('should send P2P message when activerP2P is called', () => {
//         component.activerP2P();
//         expect(socketServiceSpy.send).toHaveBeenCalledWith('message', { command: 'P2P' });
//     });
//     it('should send LAUNCH_MISSION and navigate', () => {
//         component.profileForm.controls['ipAddress'].setValue('192.168.1.1');
//         component.profileForm.controls['ipAddress2'].setValue('192.168.1.2');
//         component.profileForm.controls['mode'].setValue(ModeMission.SIMULATION); // ✅ mode valide
//         component.launch();
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.LAUNCH_MISSION, {
//             ip: '192.168.1.1',
//             ip2: '192.168.1.2',
//             mode: ModeMission.SIMULATION,
//         });
//         expect(routerSpy.navigate).toHaveBeenCalledWith(['/mission', '192.168.1.1', '192.168.1.2']);
//     });
    
//     it('should also work with PHYSIQUE mode', () => {
//         component.profileForm.controls['ipAddress'].setValue('10.0.0.1');
//         component.profileForm.controls['ipAddress2'].setValue('10.0.0.2');
//         component.profileForm.controls['mode'].setValue(ModeMission.PHYSICAL_ROBOT); 
//         component.launch();
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.LAUNCH_MISSION, {
//             ip: '10.0.0.1',
//             ip2: '10.0.0.2',
//             mode: ModeMission.PHYSICAL_ROBOT,
//         });
//         expect(routerSpy.navigate).toHaveBeenCalledWith(['/mission', '10.0.0.1', '10.0.0.2']);
//     });
    
//     it('should send P2P_OFF message when desactiverP2P is called', () => {
//         component.desactiverP2P();
//         expect(socketServiceSpy.send).toHaveBeenCalledWith('message', { command: 'P2P_OFF' });
//     });
//     it('should add STATUS and ERROR messages to serverMessages', () => {
//         // On récupère les callbacks enregistrés avec socketService.on
//         let statusCallback: (msg: string) => void;
//         let errorCallback: (msg: string) => void;
//         let missionAvailabilityCallback: (available: boolean) => void;
    
//         socketServiceSpy.on.and.callFake((event: string, cb: any) => {
//             if (event === ws_Methods.STATUS) statusCallback = cb;
//             if (event === ws_Methods.ERROR) errorCallback = cb;
//             if (event === ws_Methods.MISSION_AVAILABILITY) missionAvailabilityCallback = cb;
//         });
    
//         component.listenForServerMessages();
    
//         // Simule des messages du serveur
//         statusCallback!('Robot prêt');
//         errorCallback!('Erreur critique');
//         missionAvailabilityCallback!(false);
    
//         expect(component.serverMessages).toContain('[STATUS] Robot prêt');
//         expect(component.serverMessages).toContain('[ERROR] Erreur critique');
//         expect(component.isMissionAvailable).toBeFalse();
//     });
//     it('should set isMissionAvailable to true on MISSION_AVAILABILITY', () => {
//         socketServiceSpy.on.and.callFake((event: string, cb: any) => {
//             if (event === ws_Methods.MISSION_AVAILABILITY) {
//                 cb(true);
//             }
//         });
//         component.listenForServerMessages();
//         expect(component.isMissionAvailable).toBeTrue();
//     });  
    
//     it('should return true if mode is valid (isModeSelected)', () => {
//         component.profileForm.controls['mode'].setValue(ModeMission.PHYSICAL_ROBOT);
//         expect(component.isModeSelected).toBeTrue();
//     });
    
//     it('should return false if mode is invalid (isModeSelected)', () => {
//         component.profileForm.controls['mode'].setValue(null);
//         expect(component.isModeSelected).toBeFalse();
//     });
//     it('should return the second IP address form control', () => {
//         const control = component.secondIpAddressControl;
//         expect(control).toBe(component.profileForm.controls['ipAddress2']);
//     });
//     it('should return the main IP address form control', () => {
//         const control = component.ipAddressControl;
//         expect(control).toBe(component.profileForm.controls['ipAddress']);
//     });
//     describe('formatIPAddress', () => {
//         it('should format a valid IP with extra characters (ipAddress)', () => {
//             component.profileForm.controls['ipAddress'].setValue('192..168..0abc.300');
//             component.formatIPAddress('ipAddress');
//             expect(component.profileForm.controls['ipAddress'].value).toBe('192.168.0.255');
//         });
    
//         it('should limit each block to 3 digits and max 255', () => {
//             component.profileForm.controls['ipAddress'].setValue('999.999.999.999');
//             component.formatIPAddress('ipAddress');
//             expect(component.profileForm.controls['ipAddress'].value).toBe('255.255.255.255');
//         });
    
//         it('should trim to 4 blocks if more are present', () => {
//             component.profileForm.controls['ipAddress'].setValue('10.0.0.1.200.34');
//             component.formatIPAddress('ipAddress');
//             expect(component.profileForm.controls['ipAddress'].value).toBe('10.0.0.1');
//         });
    
//         it('should return empty if input is empty', () => {
//             component.profileForm.controls['ipAddress'].setValue('');
//             component.formatIPAddress('ipAddress');
//             expect(component.profileForm.controls['ipAddress'].value).toBe('');
//         });
    
//         it('should format ipAddress2 independently', () => {
//             component.profileForm.controls['ipAddress2'].setValue('256.256.256.256');
//             component.formatIPAddress('ipAddress2');
//             expect(component.profileForm.controls['ipAddress2'].value).toBe('255.255.255.255');
//         });

//         it('should append a dot if formattedIP matches the pattern and is short', () => {
//             component.profileForm.controls['ipAddress'].setValue('192.168.001'); // 3 blocs → regex match
//             component.formatIPAddress('ipAddress');
//             expect(component.profileForm.controls['ipAddress'].value).toBe('192.168.1.');
//         });
        
//     });
    
//     it('should send IDENTIFY for ipAddress (robot 1)', () => {
//         component.profileForm.controls['ipAddress'].setValue('192.168.0.1');
//         component.profileForm.controls['mode'].setValue(ModeMission.SIMULATION);
    
//         component.identify('ipAddress');
    
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.IDENTIFY, {
//             ip: '192.168.0.1',
//             numRobot: '1',
//             mode: ModeMission.SIMULATION,
//         });
//     });
//     it('should send IDENTIFY for ipAddress2 (robot 2)', () => {
//         component.profileForm.controls['ipAddress2'].setValue('10.0.0.2');
//         component.profileForm.controls['mode'].setValue(ModeMission.PHYSICAL_ROBOT);
    
//         component.identify('ipAddress2');
    
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.IDENTIFY, {
//             ip: '10.0.0.2',
//             numRobot: '2',
//             mode: ModeMission.PHYSICAL_ROBOT,
//         });
//     });
//     it('should handle empty ipAddress2 gracefully in launch', () => {
//         component.profileForm.controls['ipAddress'].setValue('192.168.0.1');
//         component.profileForm.controls['ipAddress2'].setValue(null); // simule champ vide
//         component.profileForm.controls['mode'].setValue(ModeMission.SIMULATION);
    
//         component.launch();
    
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.LAUNCH_MISSION, {
//             ip: '192.168.0.1',
//             ip2: '',
//             mode: ModeMission.SIMULATION,
//         });
//     });
//     it('should handle empty ipAddress gracefully in launch', () => {
//         component.profileForm.controls['ipAddress'].setValue(null);
//         component.profileForm.controls['ipAddress2'].setValue('10.0.0.2');
//         component.profileForm.controls['mode'].setValue(ModeMission.PHYSICAL_ROBOT);
    
//         component.launch();
    
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.LAUNCH_MISSION, {
//             ip: '',
//             ip2: '10.0.0.2',
//             mode: ModeMission.PHYSICAL_ROBOT,
//         });
    
//         expect(routerSpy.navigate).toHaveBeenCalledWith(['/mission', '', '10.0.0.2']);
//     });
//     it('should set isValidSecondIP = true when addSecondRobot is true and IP2 is valid', () => {
//         component.addSecondRobot = true;
//         component.profileForm.controls['ipAddress2'].setValue('192.168.1.2');
//         component.validateIP(new Event('submit'));
//         expect(component.isValidSecondIP).toBeTrue();
//     });
    
//     it('should set isValidSecondIP = false when addSecondRobot is false', () => {
//         component.addSecondRobot = false;
//         component.profileForm.controls['ipAddress2'].setValue('192.168.1.2'); // même si valide
//         component.validateIP(new Event('submit'));
//         expect(component.isValidSecondIP).toBeFalse();
//     });
//     it('should send empty IP if fieldName control is null in identify()', () => {
//         component.profileForm.controls['ipAddress2'].setValue(null); // simule champ non rempli
//         component.profileForm.controls['mode'].setValue(ModeMission.PHYSICAL_ROBOT);
        
//         component.identify('ipAddress2');
    
//         expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.IDENTIFY, {
//             ip: '', // important ici
//             numRobot: '2',
//             mode: ModeMission.PHYSICAL_ROBOT,
//         });
//     });
//     it('should return true when mode is valid (isModeSelected)', () => {
//         component.profileForm.controls['mode'].setValue(ModeMission.SIMULATION);
//         expect(component.isModeSelected).toBeTrue();
//     });
//     it('should return false when mode is null (isModeSelected)', () => {
//         component.profileForm.controls['mode'].setValue(null);
//         expect(component.isModeSelected).toBeFalse();
//     });
//     it('should return false when mode control is missing entirely (undefined case)', () => {
//         (component.profileForm as FormGroup).removeControl('mode');

    
//         expect(component.isModeSelected).toBeFalse();
//     });
    
// });
