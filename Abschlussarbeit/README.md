# LaTeX-Template Abschlussarbeit

Hier findet Ihr ein LaTeX-Template für eure Abschlussarbeit an der Fakultät efi.  
Das Template basiert auf der Vorlage von Frau Prof. Dr. Niebler und wurde  aktualisiert und ergänzt.

Die Fachschaft ist nicht dafür verantwortlich, ob das Template die aktuell gültigen formalen Bestimmungen einhält!

## Nutzung

Das Repository entweder in der Weboberfläche oder über die Kommandozeile kopieren:

    git clone --branch main https://git.efi.th-nuernberg.de/gitea/efi-fachschaft/Abschlussarbeit.git

Die Dateistruktur und Kapitelstruktur sowie die Benennung ist natürlich nicht fest und sollte so angepasst werden, wie es für eure Arbeit am Besten passt. Dazu steht im Leitfaden mehr.  
Für eine bessere Übersichtlichkeit (und evtl. Debugging) ist es definitiv ratsam, den Text logisch in verschiedene Dateien aufzuteilen.

Das Dokument wird über `00_Abschlussarbeit.tex`, `01_AbschlussarbeitPraeambel.tex` und `LTXKursTitel.sty` angepasst. Viele Optionen sind kommentiert und sollten relativ selbsterklärend sein.  
Diese Dateien daher unbedingt sorgfältig anschauen!

### LaTeX-Distribution

Ich nutze [TeX Live](https://tug.org/texlive/) als LaTeX-Distribution und Ubuntu als Betriebssystem.

### Editor

Als Editor kann ich VS Code bzw. [VS Codium](https://vscodium.com/) empfehlen (= VS Code ohne Microsoft-Binaries).

Dazu die Erweiterungen:
- LaTeX Workshop (viele nützliche Funktionen)
- LTeX (sehr gute Rechtschreib- und Grammatikprüfung)

### Kompilieren

>Kompilierreihenfolge: pdflatex > biber > makeglossaries > pdflatex > pdflatex  

In VS Code müssen dazu die Tools `biber` und `makeglossaries` ergänzt werden.  

Dazu:  
 \> Einstellungen (<kbd>Strg</kbd>+<kbd>,</kbd>)  
 \> Latex-workshop>Latex:Tools (nach "tools" suchen)  
 \> Edit in settings.json  

Diesen Code am Ende des Abschnitts "latex-workshop.latex.tools" einfügen:

    {
        "name": "biber",
        "command": "biber",
        "args": [
            "%DOCFILE%"
        ],
        "env": {}
    },
    {
        "name": "makeglossaries",
        "command": "makeglossaries",
        "args": [
            "%DOC%"
        ],
        "env": {}
    }

Anschließend muss ein entsprechendes `recipe` angelegt werden.  

Dazu:  
 \> Einstellungen (<kbd>Strg</kbd>+<kbd>,</kbd>)  
 \> Latex-workshop>Latex:Recipes (nach "recipes" suchen)  
 \> Edit in settings.json  

Diesen Code am Ende des Abschnitts "latex-workshop.latex.recipes" einfügen:

    {
       "name": "Abschlussarbeit",
       "tools": [
             "pdflatex",
             "biber",
             "makeglossaries",
             "pdflatex",
             "pdflatex"
       ]
    }

Anschließend kann man das Recipe wie andere auch normal zum Kompilieren auswählen.

Damit das verwendete Paket `glossaries` funktioniert muss außerdem [Perl](https://www.perl.org/get.html) installiert sein.

### Literatur

Literaturverwaltung funktioniert gut über [Zotero](https://www.zotero.org/), ergänzend dazu das Zotero-Plugin BetterBibTeX (automatisiert das Exportieren des Literaturverzeichnis in die .bib-Datei) und das Zotero Browser-Plugin.

### Backups

Zum Sichern und Versionieren der Arbeit bietet sich der [efi-eigene git-Server](https://git.efi.th-nuernberg.de/gitea/) an.  
Der Login erfolgt über die üblichen TH-Anmeldedaten.

## Infos, Tipps und Hilfe

Weitere Infos finden sich u.a. auf den Seiten des [Schreibzentrums](https://www.th-nuernberg.de/einrichtungen-gesamt/administration-und-service/lehr-und-kompetenzentwicklung/ueberfachliche-kompetenzen/schreibzentrum/) und der [Bibliothek](https://www.th-nuernberg.de/einrichtungen-gesamt/administration-und-service/bibliothek/).  
Feedback, Hinweise, Verbesserungsvorschläge und weitere nützliche Tools/Extensions/Plugins gerne direkt an mich:
- Hannes Dippold: dippoldha78415@th-nuernberg.de

oder die Fachschaft
- Fachschaft efi: efi-fachschaft@th-nuernberg.de
