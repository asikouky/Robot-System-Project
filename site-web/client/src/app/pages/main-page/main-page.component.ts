import { Component } from '@angular/core';
import { RouterLink } from '@angular/router';
import { FormsModule, ReactiveFormsModule } from '@angular/forms';
import { DialogAddRobotComponent } from '@app/components/dialog-add-robot/dialog-add-robot.component';
import { MenuBarComponent } from '@app/components/menu-bar/menu-bar.component';
import { AceEditorModule } from '@app/components/ace-editor/ace-editor.module'; // Importer le module
import { CommonModule } from '@angular/common';
@Component({
  selector: 'app-main-page',
  standalone: true,
  templateUrl: './main-page.component.html',
  styleUrls: ['./main-page.component.scss'],
  imports: [CommonModule,RouterLink, ReactiveFormsModule, FormsModule, DialogAddRobotComponent, MenuBarComponent, AceEditorModule] // Utiliser le module
})
export class MainPageComponent {}
