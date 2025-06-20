import http.server
import socketserver
import os
import markdown # Assicurati di aver fatto: pip3 install markdown
import webbrowser
import sys # Per sys.exit()

# --- Configurazione ---
MARKDOWN_FILENAME = "README.md"
HTML_OUTPUT_FILENAME = "README.html" # MODIFICATO: Questo sarà il file servito di default
PORT = 8000
HOST = "localhost"

# --- Funzione per convertire Markdown in HTML (identica a prima) ---
def convert_md_to_html(md_file_path, html_file_output_path):
    try:
        output_dir = os.path.dirname(html_file_output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"Creata directory: {output_dir}")

        with open(md_file_path, 'r', encoding='utf-8') as f_md:
            md_text = f_md.read()

        extensions = ['extra', 'fenced_code', 'codehilite', 'toc', 'nl2br']
        html_content = markdown.markdown(md_text, extensions=extensions)

        # Potresti voler cambiare <title> se HTML_OUTPUT_FILENAME è "index.html"
        # ma "README" va ancora bene se il contenuto è quello del README.md
        title_tag_content = os.path.splitext(os.path.basename(html_file_output_path))[0].replace("_", " ").title()
        if HTML_OUTPUT_FILENAME == "index.html" and MARKDOWN_FILENAME.endswith("README.md"):
            title_tag_content = "README"


        full_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title_tag_content}</title>
    {get_basic_styles()}
</head>
<body>
    <div class="markdown-body">
{html_content}
    </div>
</body>
</html>"""

        with open(html_file_output_path, 'w', encoding='utf-8') as f_html:
            f_html.write(full_html)
        print(f"Convertito '{md_file_path}' in '{html_file_output_path}' con successo.")
        return True
    except FileNotFoundError:
        print(f"ERRORE: File Markdown '{md_file_path}' non trovato.")
        return False
    except Exception as e:
        print(f"ERRORE durante la conversione di Markdown: {e}")
        return False

# --- Funzione per gli stili CSS (identica a prima) ---
def get_basic_styles():
    return """
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif, "Apple Color Emoji", "Segoe UI Emoji"; line-height: 1.6; margin: 0; padding: 0; background-color: #fff; }
        .markdown-body { max-width: 800px; margin: 20px auto; padding: 20px; background-color: #fff; }
        h1, h2, h3, h4, h5, h6 { margin-top: 24px; margin-bottom: 16px; font-weight: 600; line-height: 1.25; }
        h1 { font-size: 2em; padding-bottom: 0.3em; border-bottom: 1px solid #eaecef; }
        h2 { font-size: 1.5em; padding-bottom: 0.3em; border-bottom: 1px solid #eaecef; }
        p { margin-top: 0; margin-bottom: 16px; }
        ul, ol { margin-top: 0; margin-bottom: 16px; padding-left: 2em; }
        li { margin-bottom: 0.25em; }
        a { color: #0366d6; text-decoration: none; }
        a:hover { text-decoration: underline; }
        strong { font-weight: 600; }
        hr { height: 0.25em; padding: 0; margin: 24px 0; background-color: #e1e4e8; border: 0; }
        pre { padding: 16px; overflow: auto; font-size: 85%; line-height: 1.45; background-color: #f6f8fa; border-radius: 6px; margin-bottom: 16px; }
        code { font-family: "SFMono-Regular", Consolas, "Liberation Mono", Menlo, Courier, monospace; font-size: inherit; }
        table { border-collapse: collapse; margin-bottom: 16px; display: block; width: max-content; max-width: 100%; overflow: auto; }
        th, td { border: 1px solid #dfe2e5; padding: 6px 13px; }
        th { font-weight: 600; }
        img { max-width: 100%; height: auto; display: block; margin-left: auto; margin-right: auto; }
        .image-grid-table { width: 100%; margin: 25px auto; border-collapse: collapse; border-spacing: 0; border: none; }
        .image-grid-table td { width: 25%; text-align: center; vertical-align: middle; padding: 5px; border: none; }
        .image-grid-table img { max-width: 100%; max-height: 50px; height: auto; display: inline-block; }
        /* ... (resto degli stili codehilite, omessi per brevità ma inclusi nel codice originale) ... */
        .codehilite pre { margin: 0; } .codehilite .hll { background-color: #ffffcc }
        .codehilite  { background: #f8f8f8; } .codehilite .c { color: #408080; font-style: italic }
        .codehilite .err { border: 1px solid #FF0000 } .codehilite .k { color: #008000; font-weight: bold }
        .codehilite .o { color: #666666 } .codehilite .ch { color: #408080; font-style: italic }
        .codehilite .cm { color: #408080; font-style: italic } .codehilite .cp { color: #BC7A00 }
        .codehilite .cpf { color: #408080; font-style: italic } .codehilite .c1 { color: #408080; font-style: italic }
        .codehilite .cs { color: #408080; font-style: italic } .codehilite .gd { color: #A00000 }
        .codehilite .ge { font-style: italic } .codehilite .gr { color: #FF0000 }
        .codehilite .gh { color: #000080; font-weight: bold } .codehilite .gi { color: #00A000 }
        .codehilite .go { color: #888888 } .codehilite .gp { color: #000080; font-weight: bold }
        .codehilite .gs { font-weight: bold } .codehilite .gu { color: #800080; font-weight: bold }
        .codehilite .gt { color: #0044DD } .codehilite .kc { color: #008000; font-weight: bold }
        .codehilite .kd { color: #008000; font-weight: bold } .codehilite .kn { color: #008000; font-weight: bold }
        .codehilite .kp { color: #008000 } .codehilite .kr { color: #008000; font-weight: bold }
        .codehilite .kt { color: #B00040 } .codehilite .m { color: #666666 }
        .codehilite .s { color: #BA2121 } .codehilite .na { color: #7D9029 }
        .codehilite .nb { color: #008000 } .codehilite .nc { color: #0000FF; font-weight: bold }
        .codehilite .no { color: #880000 } .codehilite .nd { color: #AA22FF }
        .codehilite .ni { color: #999999; font-weight: bold } .codehilite .ne { color: #D2413A; font-weight: bold }
        .codehilite .nf { color: #0000FF } .codehilite .nl { color: #A0A000 }
        .codehilite .nn { color: #0000FF; font-weight: bold } .codehilite .nt { color: #008000; font-weight: bold }
        .codehilite .nv { color: #19177C } .codehilite .ow { color: #AA22FF; font-weight: bold }
        .codehilite .w { color: #bbbbbb } .codehilite .mb { color: #666666 }
        .codehilite .mf { color: #666666 } .codehilite .mh { color: #666666 }
        .codehilite .mi { color: #666666 } .codehilite .mo { color: #666666 }
        .codehilite .sa { color: #BA2121 } .codehilite .sb { color: #BA2121 }
        .codehilite .sc { color: #BA2121 } .codehilite .dl { color: #BA2121 }
        .codehilite .sd { color: #BA2121; font-style: italic } .codehilite .s2 { color: #BA2121 }
        .codehilite .se { color: #BB6622; font-weight: bold } .codehilite .sh { color: #BA2121 }
        .codehilite .si { color: #BB6688; font-weight: bold } .codehilite .sx { color: #008000 }
        .codehilite .sr { color: #BB6688 } .codehilite .s1 { color: #BA2121 }
        .codehilite .ss { color: #19177C } .codehilite .bp { color: #008000 }
        .codehilite .fm { color: #0000FF } .codehilite .vc { color: #19177C }
        .codehilite .vg { color: #19177C } .codehilite .vi { color: #19177C }
        .codehilite .vm { color: #19177C } .codehilite .il { color: #666666 }
    </style>
    """

# --- Funzione per avviare il server HTTP ---
def start_server():
    Handler = http.server.SimpleHTTPRequestHandler
    socketserver.TCPServer.allow_reuse_address = True

    project_root = os.path.dirname(os.path.abspath(__file__))
    os.chdir(project_root)
    print(f"Il server servirà i file dalla directory: {os.getcwd()}")

    # L'URL del server sarà http://localhost:8000/
    # Se index.html è presente nella directory di servizio,
    # SimpleHTTPRequestHandler lo servirà automaticamente.
    server_url_base = f"http://{HOST}:{PORT}"
    # Non è necessario specificare index.html nell'URL, il server lo fa di default
    # target_url = f"{server_url_base}/{HTML_OUTPUT_FILENAME}" # Meno ideale
    target_url = server_url_base # Ideale, lascia che il server trovi index.html

    with socketserver.TCPServer((HOST, PORT), Handler) as httpd:
        print(f"Server avviato su {server_url_base}")
        # Se HTML_OUTPUT_FILENAME è "index.html", aprendo server_url_base si aprirà index.html
        print(f"La pagina principale dovrebbe essere: {server_url_base}")
        print("Premi Ctrl+C per fermare il server.")

        try:
            webbrowser.open_new_tab(target_url)
        except Exception as e:
            print(f"Impossibile aprire il browser automaticamente: {e}")
            print(f"Per favore, apri manualmente: {target_url}")

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer fermato dall'utente.")
            httpd.server_close()

# --- Esecuzione principale dello script ---
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    markdown_file_full_path = os.path.join(script_dir, MARKDOWN_FILENAME)
    html_output_full_path = os.path.join(script_dir, HTML_OUTPUT_FILENAME)

    while True:
        print("\nScegli un'azione:")
        print(f"1. Converti '{MARKDOWN_FILENAME}' in '{HTML_OUTPUT_FILENAME}' e avvia server")
        print("2. Avvia server (salta conversione, usa file HTML esistente)")
        print(f"3. Solo converti '{MARKDOWN_FILENAME}' in '{HTML_OUTPUT_FILENAME}'")
        print("4. Esci")
        choice = input("Inserisci la tua scelta (1-4): ")

        if choice == '1':
            print(f"\n--- Conversione di {MARKDOWN_FILENAME} ---")
            if convert_md_to_html(markdown_file_full_path, html_output_full_path):
                print("\n--- Avvio del server ---")
                start_server()
            else:
                print("Conversione fallita. Impossibile avviare il server.")
            break
        elif choice == '2':
            print("\n--- Avvio del server (conversione saltata) ---")
            # Verifica se il file HTML di output (index.html) esiste, altrimenti avvisa
            if not os.path.exists(html_output_full_path):
                print(f"ATTENZIONE: Il file '{html_output_full_path}' non esiste.")
                print("Il server potrebbe non mostrare la pagina attesa o mostrare un elenco di directory.")
            start_server()
            break
        elif choice == '3':
            print(f"\n--- Solo conversione di {MARKDOWN_FILENAME} ---")
            convert_md_to_html(markdown_file_full_path, html_output_full_path)
            print("Operazione completata. Puoi chiudere lo script o scegliere un'altra opzione.")
        elif choice == '4':
            print("Uscita dallo script.")
            sys.exit(0)
        else:
            print("Scelta non valida. Per favore, inserisci un numero da 1 a 4.")