// /* eslint-disable */
import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionLogsComponent } from './mission-logs.component';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { environment } from 'src/environments/environment';

describe('MissionLogsComponent', () => {
  let component: MissionLogsComponent;
  let fixture: ComponentFixture<MissionLogsComponent>;
  let httpMock: HttpTestingController;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [MissionLogsComponent, HttpClientTestingModule],
    }).compileComponents();

    fixture = TestBed.createComponent(MissionLogsComponent);
    component = fixture.componentInstance;
    httpMock = TestBed.inject(HttpTestingController);

    spyOn(window, 'confirm').and.returnValue(true); 
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('devrait créer le composant', () => {
    fixture.detectChanges();
  
    
    const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs`);
    req.flush({ logs: [] });
  
    expect(component).toBeTruthy();
  });
  

  describe('#fetchMissionLogs', () => {
    it('charge les logs correctement', () => {
      fixture.detectChanges();
      const mockLogs = ['192_168_0_1_20240413_123456.log'];
      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs`);
      req.flush({ logs: mockLogs });

      expect(component.logs.length).toBe(1);
      expect(component.loading).toBeFalse();
    });

    it('gère les erreurs de requête', () => {
      fixture.detectChanges();
      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs`);
      req.error(new ErrorEvent('network error'));

      expect(component.error).toBeTrue();
      expect(component.loading).toBeFalse();
      expect(component.logs.length).toBe(0);
    });
  });

  describe('#parseLogFilenames', () => {
    it('extrait correctement IP et date du nom de fichier', () => {
      const logs = component.parseLogFilenames(['10_0_0_1_20230415_153045.log']);
      expect(logs[0].ip).toBe('10.0.0.1');
      expect(logs[0].date).toBe('15/04/2023 15:30:45');
    });

    it('gère un nom de fichier sans timestamp', () => {
      const logs = component.parseLogFilenames(['192_168_0_1.log']);
      expect(logs[0].date).toBe('Inconnue');
    });

    it('gère un format date alternative 2 parties concaténées', () => {
      const logs = component.parseLogFilenames(['192_168_0_1_20230415_153045_extra.log']);
      expect(logs[0].date).toBe('15/04/2023 15:30:45');
    });

    it('tronque les blocs IP à 4 éléments', () => {
      const logs = component.parseLogFilenames(['1_2_3_4_5_6_20231212_101010.log']);
      expect(logs[0].ip).toBe('1.2.3.4');
    });

    it('détecte une date au format compact 14 chiffres', () => {
      const logs = component.parseLogFilenames(['192_168_0_1_20240413123456.log']);
      expect(logs[0].date).toBe('13/04/2024 12:34:56');
    });
    

      
    it('utilise datePattern si la date est divisée', () => {
      const logs = component.parseLogFilenames(['192_168_0_1_20250102_112233.log']);
      expect(logs[0].date).toBe('02/01/2025 11:22:33');
    });
  });

  describe('#viewLogContent', () => {
    it('affiche le contenu d’un log', () => {
      const mockLog = {
        filename: 'test.log',
        displayName: 'Mission test',
        date: '01/01/2024 00:00:00',
        ip: '127.0.0.1',
      };

      component.viewLogContent(mockLog);

      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs/test.log`);
      req.flush({ filename: 'test.log', content: 'Contenu du log' });

      expect(component.selectedLog).toEqual(mockLog);
      expect(component.showLogContent).toBeTrue();
      expect(component.logContent).toBe('Contenu du log');
    });

    it('gère l’erreur de chargement du contenu', () => {
      const log = { filename: 'fail.log', displayName: '', date: '', ip: '' };
      component.viewLogContent(log);

      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs/fail.log`);
      req.error(new ErrorEvent('error'));

      expect(component.logContent).toContain('Erreur');
    });
  });

  describe('#deleteLog', () => {
    it('supprime un log existant', () => {
      const log = { filename: 'log1.log', displayName: 'Mission', date: '', ip: '' };
      component.logs = [log];
      component.deleteLog(log, new Event('click'));

      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs/log1.log`);
      req.flush({ message: 'deleted' });

      expect(component.logs).not.toContain(log);
    });

    it('appelle closeLogContent si le log supprimé est sélectionné', () => {
      const log = { filename: 'log1.log', displayName: 'Mission', date: '', ip: '' };
      component.logs = [log];
      component.selectedLog = log;
      const closeSpy = spyOn(component, 'closeLogContent');

      component.deleteLog(log, new Event('click'));

      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs/log1.log`);
      req.flush({ message: 'deleted' });

      expect(closeSpy).toHaveBeenCalled();
    });

    it('gère les erreurs lors de la suppression', () => {
      spyOn(window, 'alert');

      const log = { filename: 'error.log', displayName: 'Erreur', date: '', ip: '' };
      component.logs = [log];
      component.deleteLog(log, new Event('click'));

      const req = httpMock.expectOne(`${environment.serverUrl}/api/mission-logs/error.log`);
      req.error(new ErrorEvent('fail'));

      expect(window.alert).toHaveBeenCalledWith('Échec de la suppression du journal');
    });

    it('ne supprime pas si l’utilisateur annule', () => {
      (window.confirm as jasmine.Spy).and.returnValue(false);
      const log = { filename: 'cancel.log', displayName: '', date: '', ip: '' };
      component.logs = [log];
      component.deleteLog(log, new Event('click'));

      httpMock.expectNone(`${environment.serverUrl}/api/mission-logs/cancel.log`);
    });
  });

  describe('#closeLogContent', () => {
    it('cache le contenu du log', () => {
      component.selectedLog = { filename: 'file', displayName: '', date: '', ip: '' };
      component.showLogContent = true;

      component.closeLogContent();

      expect(component.selectedLog).toBeNull();
      expect(component.showLogContent).toBeFalse();
    });
  });

  describe('#refreshLogs', () => {
    it('relance fetchMissionLogs()', () => {
      spyOn(component, 'fetchMissionLogs');
      component.refreshLogs();
      expect(component.fetchMissionLogs).toHaveBeenCalled();
    });
  });


  
});
