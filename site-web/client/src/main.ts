import { provideHttpClient } from '@angular/common/http';
import { enableProdMode, importProvidersFrom } from '@angular/core';
import { bootstrapApplication } from '@angular/platform-browser';
import { provideAnimations } from '@angular/platform-browser/animations';
import { Routes, provideRouter } from '@angular/router';
import { MissionListComponent } from '@app/components/mission-list/mission-list.component';
import { AppMaterialModule } from '@app/modules/material.module';
import { AppComponent } from '@app/pages/app/app.component';
import { MainPageComponent } from '@app/pages/main-page/main-page.component';
import { MaterialPageComponent } from '@app/pages/material-page/material-page.component';
import { MissionLogsComponent } from '@app/pages/mission-logs/mission-logs.component';
import { MissionPageComponent } from '@app/pages/mission-page/mission-page.component';
import { environment } from './environments/environment';
import { MapComponent } from '@app/components/map/map/map.component';

if (environment.production) {
  enableProdMode();
}

const routes: Routes = [
  { path: '', redirectTo: '/home', pathMatch: 'full' },
  { path: 'home', component: MainPageComponent },
  { path: 'mission/:ip/:ip2', component: MissionPageComponent}, 
    //resolve: { mission: MissionResolver }},
  { path: 'material', component: MaterialPageComponent },
  { path: 'mission-logs', component: MissionLogsComponent }, // Add this line
  { path: 'map', component: MapComponent },
  {path: 'mission-list', component: MissionListComponent},
  { path: '**', redirectTo: '/home' },
  // Add this line
];

bootstrapApplication(AppComponent, {
  providers: [provideHttpClient(), provideRouter(routes), provideAnimations(), importProvidersFrom(AppMaterialModule)],
});
