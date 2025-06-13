
import { Component, OnInit } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MenuBarComponent } from '../menu-bar/menu-bar.component';
import { ModeMission } from '@common/events';
export interface Mission {
  _id: string;
  missionId: string;
  ip: string;
  ip2?: string;
  status: string;
  createdAt: string;
  mode: ModeMission;
  duration: number;
  distanceTravelled: {
    [robotIp: string]: number; 
  };
  mapImage?: string | null;
  logFilename?: string;
  logHistory: string[] ;

}


@Component({
  selector: 'app-mission-list',
  templateUrl: './mission-list.component.html',
  standalone: true,
  imports: [CommonModule, FormsModule, MenuBarComponent],
  styleUrls:["./mission-list.component.scss"]
 
  

})
export class MissionListComponent implements OnInit {
  missions: Mission[] = [];
  selectedMission?: Mission;
  showMissionDetail: boolean = false;
  sortField: 'date' | 'ip' | 'mode' = 'date';
  sortDirection: 'asc' | 'desc' = 'desc';
  filterIp: string = '';
  filterMode: string = 'all'; 

  constructor(private http: HttpClient) {}

  ngOnInit(): void {
    this.loadMissions();
  }
  loadMissions(): void {
    this.http.get<Mission[]>('http://localhost:3000/api/mission').subscribe({
      next: (data) => {
        this.missions = data;
       
      },
      error: (err) => console.error('Erreur de chargement :', err)
    });
  }
  selectMission(mission: Mission): void {
    this.selectedMission = mission;
  }
  

viewMissionDetail(mission: any): void {
  this.selectedMission = mission;
  this.showMissionDetail = true;
}

closeMissionDetail(): void {
  this.selectedMission = undefined;
  this.showMissionDetail = false;
}

  
  getTotalDistance(mission: Mission): number {
    if (!mission.distanceTravelled) return 0;
    return Object.values(mission.distanceTravelled).reduce((a, b) => a + b, 0);
  }
  
  deleteMission(id: string): void {
    if (!confirm('Supprimer cette mission ?')) return;

    this.http.delete(`http://localhost:3000/api/mission/${id}`).subscribe({
      next: () => this.loadMissions(),
      error: (err) => console.error('Erreur de suppression :', err)
    });
}

get sortedMissions(): Mission[] {
  return this.missions
    .filter(m => {
      const ipMatch = this.filterIp === '' || m.ip.includes(this.filterIp);
      const modeMatch = this.filterMode === 'all' || m.mode === this.filterMode;
      return ipMatch && modeMatch;
    })
    .slice()
    .sort((a, b) => {
      let valA, valB;
      switch (this.sortField) {
        case 'date':
          valA = new Date(a.createdAt).getTime();
          valB = new Date(b.createdAt).getTime();
          break;
        case 'mode':
          valA = a.mode.toLowerCase();
          valB = b.mode.toLowerCase();
          break;
        case 'ip':
          valA = a.ip;
          valB = b.ip;
          break;
      }
      if (valA < valB) return this.sortDirection === 'asc' ? -1 : 1;
      if (valA > valB) return this.sortDirection === 'asc' ? 1 : -1;
      return 0;
    });
}

toggleSortDirection(): void {
  this.sortDirection = this.sortDirection === 'asc' ? 'desc' : 'asc';
}
downloadLogJson(logLines: string[]): void {
  const data = {
    missionId: this.selectedMission?.missionId,
    date: this.selectedMission?.createdAt,
    mode: this.selectedMission?.mode,
    ip: this.selectedMission?.ip,
    ip2: this.selectedMission?.ip2,
    duration: this.selectedMission?.duration,
    status: this.selectedMission?.status,
    logs: logLines.map((line, index) => ({
      id: index + 1,
      timestamp: new Date().toISOString(), // facultatif
      raw: line
    }))
  };

  const blob = new Blob([JSON.stringify(data, null, 2)], {
    type: 'application/json'
  });
  const url = window.URL.createObjectURL(blob);

  const a = document.createElement('a');
  a.href = url;
  a.download = 'mission_log.json';
  a.click();

  window.URL.revokeObjectURL(url);
}


}
