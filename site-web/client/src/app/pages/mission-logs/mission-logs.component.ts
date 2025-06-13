import { CommonModule } from '@angular/common';
import { HttpClient } from '@angular/common/http';
import { Component, OnInit } from '@angular/core';
import { MenuBarComponent } from '@app/components/menu-bar/menu-bar.component';
import { of } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { environment } from 'src/environments/environment';

interface MissionLog {
  filename: string;
  displayName: string;
  date: string;
  ip: string;
}

@Component({
  selector: 'app-mission-logs',
  templateUrl: './mission-logs.component.html',
  styleUrls: ['./mission-logs.component.scss'],
  standalone: true,
  imports: [CommonModule, MenuBarComponent]
})
export class MissionLogsComponent implements OnInit {
  logs: MissionLog[] = [];
  loading = true;
  error = false;
  selectedLog: MissionLog | null = null;
  logContent: string = '';
  showLogContent = false;

  constructor(private http: HttpClient) { }

  ngOnInit(): void {
    this.fetchMissionLogs();
  }

  fetchMissionLogs(): void {
    this.loading = true;
    this.error = false;
    
    this.http.get<{ logs: string[] }>(`${environment.serverUrl}/api/mission-logs`)
      .pipe(
        catchError(error => {
          console.error('Erreur lors de la récupération des journaux de mission:', error);
          this.error = true;
          this.loading = false;
          return of({ logs: [] });
        })
      )
      .subscribe(response => {
        this.logs = this.parseLogFilenames(response.logs);
        this.loading = false;
      });
  }

  parseLogFilenames(filenames: string[]): MissionLog[] {
    return filenames.map(filename => {
      const parts = filename.replace('.log', '').split('_');
      
      const ipParts: string[] = [];
      
      const ipLength = 4; // Nombre standard de segments pour une adresse IPv4
      for (let i = 0; i < Math.min(ipLength, parts.length); i++) {
        ipParts.push(parts[i]);
      }
      
      const ip = ipParts.join('.');
      
      let date = 'Inconnue';
      const datePattern = /^(\d{8})(\d{6})$/;
      
      for (let i = ipLength; i < parts.length; i++) {
        if (parts[i].length === 14 && !isNaN(Number(parts[i]))) {
          const dateTimeStr = parts[i];
          
          const year = dateTimeStr.substring(0, 4);
          const month = dateTimeStr.substring(4, 6);
          const day = dateTimeStr.substring(6, 8);
          
          const hour = dateTimeStr.substring(8, 10);
          const minute = dateTimeStr.substring(10, 12);
          const second = dateTimeStr.substring(12, 14);
          
          date = `${day}/${month}/${year} ${hour}:${minute}:${second}`;
          break;
        } else if (datePattern.test(parts[i] + (parts[i+1] || ''))) {

          const matches = datePattern.exec(parts[i] + (parts[i+1] || ''));
          if (matches) {
            const dateStr = matches[1]; // YYYYMMDD
            const timeStr = matches[2]; // HHMMSS
            
            const year = dateStr.substring(0, 4);
            const month = dateStr.substring(4, 6);
            const day = dateStr.substring(6, 8);
            
            const hour = timeStr.substring(0, 2);
            const minute = timeStr.substring(2, 4);
            const second = timeStr.substring(4, 6);
            
            date = `${day}/${month}/${year} ${hour}:${minute}:${second}`;
            break;
          }
        }
      }

      const displayName = `Mission : ${ip} - ${date}`;

      return {
        filename,
        displayName,
        date,
        ip
      };
    });
  }

  viewLogContent(log: MissionLog): void {
    this.selectedLog = log;
    this.showLogContent = true;
    
    this.http.get<{ filename: string, content: string }>(`${environment.serverUrl}/api/mission-logs/${log.filename}`)
      .pipe(
        catchError(error => {
          console.error('Erreur lors de la récupération du contenu du journal:', error);
          return of({ filename: log.filename, content: 'Erreur lors du chargement du contenu du journal' });
        })
      )
      .subscribe(response => {
        this.logContent = response.content;
      });
  }

  closeLogContent(): void {
    this.showLogContent = false;
    this.selectedLog = null;
  }

  deleteLog(log: MissionLog, event: Event): void {
    event.stopPropagation();
    
    if (confirm(`Êtes-vous sûr de vouloir supprimer le journal : ${log.displayName} ?`)) {
      this.http.delete<{ message: string }>(`${environment.serverUrl}/api/mission-logs/${log.filename}`)
        .pipe(
          catchError(error => {
            console.error('Erreur lors de la suppression du journal:', error);
            alert('Échec de la suppression du journal');
            return of({ message: 'Erreur' });
          })
        )
        .subscribe(response => {
          if (response.message !== 'Erreur') {
            this.logs = this.logs.filter(l => l.filename !== log.filename);
            
            if (this.selectedLog && this.selectedLog.filename === log.filename) {
              this.closeLogContent();
            }
          }
        });
    }
  }

  refreshLogs(): void {
    this.fetchMissionLogs();
  }
}
