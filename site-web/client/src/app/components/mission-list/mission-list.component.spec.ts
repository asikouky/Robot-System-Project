import { ComponentFixture, TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { MissionListComponent, Mission } from './mission-list.component';
import { ModeMission } from '@common/events';

describe('MissionListComponent', () => {
  let component: MissionListComponent;
  let fixture: ComponentFixture<MissionListComponent>;
  let httpMock: HttpTestingController;

  const mockMissions: Mission[] = [
    {
      _id: '1',
      missionId: 'mission-1',
      ip: '192.168.1.1',
      ip2: undefined,
      status: 'IN_MISSION',
      createdAt: '2025-04-14T12:00:00Z',
      mode: ModeMission.SIMULATION,
      duration: 120,
      distanceTravelled: { '192.168.1.1': 15 },
      mapImage: null,
      logFilename: 'mission-1.log',
      logHistory: []
    },
    {
      _id: '2',
      missionId: 'mission-2',
      ip: '192.168.1.2',
      ip2: undefined,
      status: 'IDLE',
      createdAt: '2025-04-14T13:00:00Z',
      mode: ModeMission.PHYSICAL_ROBOT,
      duration: 90,
      distanceTravelled: { '192.168.1.2': 10 },
      logFilename: 'mission-2.log',
      logHistory: []
    }
  ];
  

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [HttpClientTestingModule], 
    }).compileComponents();

    fixture = TestBed.createComponent(MissionListComponent);
    component = fixture.componentInstance;
    httpMock = TestBed.inject(HttpTestingController);
    fixture.detectChanges();
    const req = httpMock.expectOne('http://localhost:3000/api/mission');
  req.flush([]); 
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('devrait créer le composant', () => {
    expect(component).toBeTruthy();
  });

  it('devrait charger les missions via HTTP', () => {
    component.loadMissions();

    const req = httpMock.expectOne('http://localhost:3000/api/mission');
    expect(req.request.method).toBe('GET');
    req.flush(mockMissions);

    expect(component.missions.length).toBe(2);
    expect(component.missions).toEqual(mockMissions);
  });

  it('devrait sélectionner une mission', () => {
    const mission = mockMissions[0];
    component.selectMission(mission);

    expect(component.selectedMission).toEqual(mission);
  });

  it('devrait afficher les détails d\'une mission', () => {
    const mission = mockMissions[0];
    component.viewMissionDetail(mission);

    expect(component.selectedMission).toEqual(mission);
    expect(component.showMissionDetail).toBeTrue();
  });

  it('devrait fermer les détails d\'une mission', () => {
    component.viewMissionDetail(mockMissions[0]);
    component.closeMissionDetail();

    expect(component.selectedMission).toBeUndefined();
    expect(component.showMissionDetail).toBeFalse();
  });

  it('devrait calculer la distance totale parcourue', () => {
    const mission = mockMissions[0];
    const totalDistance = component.getTotalDistance(mission);

    expect(totalDistance).toBe(15);
  });

  it('devrait supprimer une mission via HTTP', () => {
    spyOn(window, 'confirm').and.returnValue(true); 

    component.missions = [...mockMissions];

    component.deleteMission('1');

    const req = httpMock.expectOne('http://localhost:3000/api/mission/1');
    expect(req.request.method).toBe('DELETE');
    req.flush({}); 
   
    const reloadReq = httpMock.expectOne('http://localhost:3000/api/mission');
    reloadReq.flush(mockMissions.slice(1)); 
    expect(component.missions.length).toBe(1);
    expect(component.missions[0]._id).toBe('2');
  });

  it('ne devrait pas supprimer une mission si l’utilisateur annule la confirmation', () => {
    spyOn(window, 'confirm').and.returnValue(false); 
    component.missions = [...mockMissions];

    component.deleteMission('1');

    httpMock.expectNone('http://localhost:3000/api/mission/1');
    expect(component.missions.length).toBe(2);
  });
});
