const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

// Spawn C++ process
// Assuming we are in /web and exe is in /build
const exePath = path.join(__dirname, '../build/pathfinder_viz.exe');
console.log(`Spawning C++ process: ${exePath}`);

let isCppReady = false;

const cppProcess = spawn(exePath, ['--server'], {
    stdio: ['pipe', 'pipe', 'pipe']
});

cppProcess.on('error', (err) => {
    console.error('Failed to start C++ process:', err);
});

cppProcess.stderr.on('data', (data) => {
    const msg = data.toString();
    console.error(`CPP Log: ${msg}`);
    if (msg.includes('Server Ready')) {
        isCppReady = true;
        console.log('C++ Backend is Ready');
    }
});

cppProcess.on('close', (code) => {
    console.log(`CPP process exited with code ${code}`);
    process.exit(code);
});

// Helper to send command and wait for specific response
let pendingRequest = null;
let requestQueue = [];
let buffer = '';

function processQueue() {
    if (pendingRequest) return; // Already processing
    if (requestQueue.length === 0) return; // Nothing to do

    const next = requestQueue.shift();
    pendingRequest = next;
    
    // Execute logic based on type
    if (next.type === 'benchmark') {
        const data = next.data;
        let cmd = `B ${data.start.q} ${data.start.r} ${data.end.q} ${data.end.r}`;
        if (data.targets && Array.isArray(data.targets) && data.targets.length > 0) {
            cmd += ' ' + data.targets.join(' ');
        }
        cmd += '\n';
        cppProcess.stdin.write(cmd);
    } else if (next.type === 'path') {
        const data = next.data;
        const algo = data.algo || 'AStar';
        const cmd = `P ${data.start.q} ${data.start.r} ${data.end.q} ${data.end.r} ${algo}\n`;
        cppProcess.stdin.write(cmd);
    } else if (next.type === 'obstacle') {
        const data = next.data;
        const cmd = `O ${data.q} ${data.r} ${data.state ? 1 : 0}\n`;
        cppProcess.stdin.write(cmd);
    } else if (next.type === 'load') {
        const body = next.data; // Raw body string
        cppProcess.stdin.write('L ' + body + '\n');
    } else if (next.type === 'generate') {
        const data = next.data;
        let cmd = 'G\n';
        if (data && data.width && data.height) {
            cmd = `G ${data.width} ${data.height}\n`;
        }
        cppProcess.stdin.write(cmd);
    }
}

cppProcess.stdout.on('data', (data) => {
    const chunk = data.toString();
    buffer += chunk;
    
    // Debug logging for Benchmark
    if (pendingRequest && pendingRequest.type === 'benchmark') {
        console.log('Buffer Update:', chunk);
    }
    
    // Check for Map Data
    if (pendingRequest && pendingRequest.type === 'generate') {
        const marker = 'MAP_DATA ';
        const idx = buffer.indexOf(marker);
        if (idx !== -1) {
            // Check for newline at the end indicating completion
            if (buffer.endsWith('\n') || buffer.endsWith('\r')) {
                const content = buffer.substring(idx + marker.length).trim(); 
                
                // Log what was before it (Debug)
                if (idx > 0) {
                     // console.log('CPP Logs ignored:', buffer.substring(0, idx));
                }

                pendingRequest.res.writeHead(200, { 'Content-Type': 'text/plain' });
                pendingRequest.res.end(content);
                pendingRequest = null;
                buffer = '';
                processQueue();
            }
        }
    }

    // Check for OK response
    if (buffer.includes('OK\n') || buffer.includes('OK\r\n')) {
        const lines = buffer.split(/\r?\n/);
        buffer = ''; // Simple clear, assuming one cmd at a time
        
        if (pendingRequest) {
            if (pendingRequest.type === 'obstacle' || pendingRequest.type === 'load') {
                pendingRequest.res.writeHead(200, { 'Content-Type': 'application/json' });
                pendingRequest.res.end(JSON.stringify({ status: 'ok' }));
                pendingRequest = null;
                processQueue();
            }
        }
    }

    // Check for Benchmark Result
    if (buffer.includes('BENCH_END')) {
        let startIdx = buffer.indexOf('BENCH_START');
        const endIdx = buffer.indexOf('BENCH_END');
        
        // Robustness: If BENCH_START is missing (maybe cleared), look for JSON array start
        if (startIdx === -1) {
             startIdx = buffer.lastIndexOf('[', endIdx);
        }

        if (startIdx !== -1 && endIdx !== -1) {
            // Extract JSON array between lines
            const jsonStart = buffer.indexOf('[', startIdx);
            const jsonEnd = buffer.lastIndexOf(']', endIdx) + 1;
            
            if (jsonStart !== -1 && jsonEnd !== -1) {
                const jsonStr = buffer.substring(jsonStart, jsonEnd);
                if (pendingRequest && pendingRequest.type === 'benchmark') {
                    pendingRequest.res.writeHead(200, { 'Content-Type': 'application/json' });
                    pendingRequest.res.end(jsonStr);
                    pendingRequest = null;
                    processQueue();
                }
            }
            // Clear buffer up to end
            buffer = buffer.substring(endIdx + 9).trim(); // 9 = len("BENCH_END")
        }
    }

    // Check for Path Result
    if (buffer.includes('RESULT_END')) {
        const lines = buffer.split(/\r?\n/);
        buffer = '';
        
        let result = { path: [], visited: [], found: false };
        
        for (const line of lines) {
            if (line.startsWith('PATH')) {
                const parts = line.split(' ');
                const count = parseInt(parts[1]);
                for (let i = 0; i < count; i++) {
                    const coords = parts[2 + i].split(',');
                    if (coords.length === 2) {
                        result.path.push({ q: parseInt(coords[0]), r: parseInt(coords[1]) });
                    }
                }
                result.found = true;
            } else if (line.startsWith('VISITED')) {
                const parts = line.split(' ');
                const count = parseInt(parts[1]);
                for (let i = 0; i < count; i++) {
                    const coords = parts[2 + i].split(',');
                    if (coords.length === 2) {
                        result.visited.push({ q: parseInt(coords[0]), r: parseInt(coords[1]) });
                    }
                }
            } else if (line.startsWith('NOPATH')) {
                result.found = false;
            }
        }

        if (pendingRequest && pendingRequest.type === 'path') {
            pendingRequest.res.writeHead(200, { 'Content-Type': 'application/json' });
            pendingRequest.res.end(JSON.stringify(result));
            pendingRequest = null;
            processQueue();
        }
    }
});

const server = http.createServer((req, res) => {
    console.log(`[Request] ${req.method} ${req.url}`);
    
    // Parse URL
    const urlObj = new URL(req.url, `http://${req.headers.host}`);
    const pathname = urlObj.pathname;

    // Serve HTML
    if (req.method === 'GET' && (pathname === '/' || pathname === '/index.html')) {
        fs.readFile(path.join(__dirname, 'index.html'), (err, content) => {
            if (err) {
                res.writeHead(500);
                res.end('Error loading index.html');
            } else {
                res.writeHead(200, { 'Content-Type': 'text/html' });
                res.end(content);
            }
        });
        return;
    }

    // API
    if (req.method === 'POST' && pathname === '/api/path') {
        if (!isCppReady) {
            res.writeHead(503);
            res.end('Initializing');
            return;
        }
        let body = '';
        req.on('data', chunk => body += chunk);
        req.on('end', () => {
            let data;
            try {
                data = JSON.parse(body);
            } catch (e) {
                res.writeHead(400);
                res.end('Invalid JSON');
                return;
            }

            // If algo is 'BENCHMARK', send B command
            if (data.algo === 'BENCHMARK') {
                requestQueue.push({ type: 'benchmark', res, data });
            } else {
                requestQueue.push({ type: 'path', res, data });
            }
            processQueue();
        });
        return;
    }

    if (req.method === 'POST' && pathname === '/api/obstacle') {
        if (!isCppReady) {
            res.writeHead(503);
            res.end('Initializing');
            return;
        }
        let body = '';
        req.on('data', chunk => body += chunk);
        req.on('end', () => {
            let data;
            try {
                data = JSON.parse(body);
            } catch (e) {
                res.writeHead(400);
                res.end('Invalid JSON');
                return;
            }
            requestQueue.push({ type: 'obstacle', res, data });
            processQueue();
        });
        return;
    }

    if (req.method === 'POST' && pathname === '/api/load') {
        if (!isCppReady) {
            res.writeHead(503);
            res.end('Initializing');
            return;
        }
        
        let body = '';
        req.on('data', chunk => body += chunk);
        req.on('end', () => {
             // Body should be "width height count val count val..."
             // We need to prefix "L "
             requestQueue.push({ type: 'load', res, data: body });
             processQueue();
        });
        return;
    }

    if (req.method === 'POST' && pathname === '/api/generate') {
        if (!isCppReady) {
            res.writeHead(503);
            res.end('Initializing');
            return;
        }
        
        let body = '';
        req.on('data', chunk => body += chunk);
        req.on('end', () => {
             let data = null;
             if (body) {
                 try {
                     data = JSON.parse(body);
                 } catch (e) {}
             }
             requestQueue.push({ type: 'generate', res, data });
             processQueue();
        });
        return;
    }

    res.writeHead(404);
    res.end(`Not Found: ${pathname}`);
});

server.listen(8080, () => {
    console.log('Server running at http://localhost:8080/');
});
