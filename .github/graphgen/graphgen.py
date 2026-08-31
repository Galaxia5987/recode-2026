import os
import subprocess
import secrets
import base64
import json
import zlib

def js_string_to_byte(data):
    return bytes(data, 'ascii')

def js_bytes_to_string(data):
    return data.decode('ascii')

def js_btoa(data):
    return base64.b64encode(data)

def pako_deflate(data):
    compress = zlib.compressobj(9, zlib.DEFLATED, 15, 8,zlib.Z_DEFAULT_STRATEGY)
    compressed_data = compress.compress(data)
    compressed_data += compress.flush()
    return compressed_data

def gen_pako_link(graph_markdown: str):
    j_graph = {
        "code": graph_markdown,
        "mermaid": {"theme": "default"}
    }
    byte_str = js_string_to_byte(json.dumps(j_graph))
    deflated = pako_deflate(byte_str)
    d_encode = js_btoa(deflated)
    link = 'http://mermaid.live/view#pako:' + js_bytes_to_string(d_encode).replace('+', '-').replace('/', '_')
    return link

def get_changed_graph_files(pr_number, repo):
    cmd = [
        "gh", "pr", "view", pr_number,
        "--repo", repo,
        "--json", "files",
        "--jq", ".files[].path"
    ]

    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        return []

    files = result.stdout.strip().split('\n')

    return [
        f for f in files
        if f.startswith("graphs/") and f.endswith(".md") and os.path.isfile(f)
    ]

def main():
    max_size = 64000
    pr_number = os.environ.get("PR_NUMBER")
    repo = os.environ.get("GITHUB_REPOSITORY")
    github_output = os.environ.get("GITHUB_OUTPUT")

    if not pr_number or not repo:
        raise ValueError("PR_NUMBER and GITHUB_REPOSITORY environment variables are required.")

    comment_body = "## State Machine Graphs\nGenerated from the latest commit.\n\n"
    truncate_msg = "> Note: Comment truncated due to GitHub size limits. Some graphs are omitted.\n"

    for file_path in get_changed_graph_files(pr_number, repo):
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        mermaid_code = '\n'.join(content.split("\n")[1:-1])
        chunk = f"### `{file_path}`\n[Interactive Link]({gen_pako_link(mermaid_code)})\n{content}\n\n"

        # Measure byte size to match GitHub's limits
        if len(comment_body.encode('utf-8')) + len(chunk.encode('utf-8')) > max_size:
            comment_body += truncate_msg
            break

        comment_body += chunk

    if github_output:
        delimiter = secrets.token_hex(8)
        with open(github_output, "a", encoding="utf-8") as f:
            f.write(f"comment_body<<{delimiter}\n")
            f.write(comment_body)
            if not comment_body.endswith('\n'):
                f.write('\n')
            f.write(f"{delimiter}\n")
    else:
        print(comment_body)

if __name__ == "__main__":
    main()