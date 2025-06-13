// AceEditorComponent (frontend)
import { Component, OnInit, ElementRef } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as ace from 'ace-builds';

@Component({
    selector: 'app-ace-editor',
    templateUrl: './ace-editor.component.html',
    styleUrls: ['./ace-editor.component.scss'],
})
export class AceEditorComponent implements OnInit {
    editor: any;

    constructor(
        private el: ElementRef,
        private http: HttpClient,
        
    ) {}

    ngOnInit(): void {
        
        ace.config.set('basePath', 'https://cdnjs.cloudflare.com/ajax/libs/ace/1.4.14/');

        this.editor = ace.edit(this.el.nativeElement.querySelector('#editor'));
        this.editor.setTheme('ace/theme/monokai'); 
        this.editor.session.setMode('ace/mode/python'); 

        
        this.loadCode();
    }

   
    loadCode() {
        this.http.get('http://localhost:3000/api/robot/getCode', { responseType: 'text' }).subscribe(
            (data) => {
                this.editor.setValue(data); 
                this.editor.clearSelection(); 
            },
            (error) => {
                console.error('Erreur lors du chargement du code :', error);
            },
        );
    }

    
    saveCode() {
        const code = this.editor.getValue(); 
        console.log('Code sauvegardé :', code);

        
        this.http.post('http://localhost:3000/api/robot/saveCode', { code }).subscribe(
            (response) => {
                console.log('Code envoyé avec succès !');

               
                this.restartGazebo();
            },
            (error) => {
                console.error('Erreur lors de la sauvegarde :', error);
            },
        );
    }

    // Redémarrer Gazebo après la sauvegarde
    restartGazebo() {
        this.http.post('http://localhost:3000/api/robot/restartGazebo', {}).subscribe(
            (response) => {
                console.log('Gazebo redémarré avec succès !');
            },
            (error) => {
                console.error('Erreur lors du redémarrage de Gazebo :', error);
            },
        );
    }
}
