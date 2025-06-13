import { CommonModule } from '@angular/common';
import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { AceEditorModule } from '@app/components/ace-editor/ace-editor.module'; // Importer le module ici

import { MissionListComponent } from '../mission-list/mission-list.component';

@Component({
    selector: 'app-menu-bar',
    standalone: true,
    imports: [CommonModule, MissionListComponent, AceEditorModule],
    templateUrl: './menu-bar.component.html',
    styleUrls: ['./menu-bar.component.scss'],
})

export class MenuBarComponent {
    isMenuOpen = false;

    constructor(private router: Router) {}

    navigateTo(route: string) {
        this.router.navigate([route]);
        console.log('Navigating to:', route);
        this.closeMenu();
    }

    toggleMenu() {
        this.isMenuOpen = !this.isMenuOpen;
        this.updateMenuState();
    }

    closeMenu() {
        this.isMenuOpen = false;
        this.updateMenuState();
    }

    private updateMenuState() {
        const menuList = document.getElementById('menuList');
        const overlay = document.getElementById('overlay');
        const mainContainer = document.getElementById('mainContainer');

        if (menuList && overlay) {
            if (this.isMenuOpen) {
                menuList.classList.add('show');
                overlay.classList.add('active');
            } else {
                menuList.classList.remove('show');
                overlay.classList.remove('active');
            }
        }

        if (this.router.url === '/mission' && mainContainer) {
            if (this.isMenuOpen) {
                mainContainer.classList.add('blurred', 'no-interaction');
            } else {
                mainContainer.classList.remove('blurred', 'no-interaction');
            }
        }
    }

   
    openEditor() {
      const codeEditorOverlay = document.getElementById('codeEditorOverlay');
      const overlay = document.getElementById('overlay');
      if (codeEditorOverlay && overlay) {
        codeEditorOverlay.classList.add('active');
        overlay.classList.add('active');
      }
    }
  
   
    closeEditor() {
      const codeEditorOverlay = document.getElementById('codeEditorOverlay');
      const overlay = document.getElementById('overlay');
      if (codeEditorOverlay && overlay) {
        codeEditorOverlay.classList.remove('active');
        overlay.classList.remove('active');
      }
    }
  
   
    saveCode() {
      console.log('Code sauvegardé');
      
    }
}
