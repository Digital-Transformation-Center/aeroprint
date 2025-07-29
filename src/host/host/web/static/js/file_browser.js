// file_browser.js
const fileList = document.getElementById('file-list');
const breadcrumbList = document.getElementById('breadcrumb-list');
const refreshBtn = document.getElementById('refresh-btn');
const deleteAllBtn = document.getElementById('delete-all-btn');

let currentPath = '';

function fetchFiles(path = '') {
  axios.get(`/api/list_assets/${path}`)
    .then(res => {
      renderFiles(res.data, path);
      renderBreadcrumb(path);
    })
    .catch(err => {
      fileList.innerHTML = `<li class="list-group-item text-danger">Error loading files: ${err.response?.data?.error || err.message}</li>`;
    });
}

function renderFiles(files, path) {
  fileList.innerHTML = '';
  if (path) {
    // Add up navigation
    const upPath = path.split('/').slice(0, -1).join('/');
    const upLi = document.createElement('li');
    upLi.className = 'list-group-item list-group-item-action';
    upLi.innerHTML = '<b>.. (Up)</b>';
    upLi.onclick = () => navigateTo(upPath);
    fileList.appendChild(upLi);
  }
  files.forEach(item => {
    const li = document.createElement('li');
    li.className = 'list-group-item';
    const row = document.createElement('div');
    row.className = 'd-flex justify-content-between align-items-center';
    if (item.type === 'directory') {
      const left = document.createElement('span');
      left.innerHTML = `📁 <a href="#">${item.name}</a>`;
      left.querySelector('a').onclick = (e) => {
        e.preventDefault();
        navigateTo(item.path.replace('/api/list_assets/', ''));
      };
      // Add right-click context menu for delete
      left.addEventListener('contextmenu', function(e) {
        e.preventDefault();
        // Remove any existing context menu
        const oldMenu = document.getElementById('dir-context-menu');
        if (oldMenu) oldMenu.remove();
        // Create menu
        const menu = document.createElement('div');
        menu.id = 'dir-context-menu';
        menu.style.position = 'absolute';
        menu.style.zIndex = 10000;
        menu.style.left = e.pageX + 'px';
        menu.style.top = e.pageY + 'px';
        menu.style.background = '#fff';
        menu.style.border = '1px solid #ccc';
        menu.style.padding = '4px 0';
        menu.style.boxShadow = '0 2px 8px rgba(0,0,0,0.15)';
        menu.style.minWidth = '120px';
        const delOpt = document.createElement('div');
        delOpt.textContent = 'Delete';
        delOpt.style.padding = '4px 16px';
        delOpt.style.cursor = 'pointer';
        delOpt.onmouseover = () => delOpt.style.background = '#eee';
        delOpt.onmouseout = () => delOpt.style.background = '';
        delOpt.onclick = (ev) => {
          menu.remove();
          if (/^\d+$/.test(item.name)) {
            deleteFile(`/assets/${item.name}`);
          } else {
            alert('Only numbered asset folders can be deleted via this interface.');
          }
          ev.stopPropagation();
        };
        menu.appendChild(delOpt);
        document.body.appendChild(menu);
        // Remove menu on click elsewhere, with a small delay to avoid immediate removal
        setTimeout(() => {
          document.addEventListener('mousedown', function handler() {
            menu.remove();
            document.removeEventListener('mousedown', handler);
          });
        }, 0);
      });
      row.appendChild(left);
      delBtn.className = 'btn btn-danger btn-sm';
      delBtn.textContent = 'Delete';
      delBtn.onclick = () => deleteFile(item.path);
      row.appendChild(delBtn);
    } else {
      const left = document.createElement('span');
      left.innerHTML = `📄 <a href="${item.path}" target="_blank">${item.name}</a>`;
      row.appendChild(left);
      const delBtn = document.createElement('button');
      delBtn.className = 'btn btn-danger btn-sm';
      delBtn.textContent = 'Delete';
      delBtn.onclick = () => deleteFile(item.path);
      row.appendChild(delBtn);
    }
    li.appendChild(row);
    fileList.appendChild(li);
  });
}

function renderBreadcrumb(path) {
  breadcrumbList.innerHTML = '';
  const parts = path ? path.split('/') : [];
  let accum = '';
  const rootLi = document.createElement('li');
  rootLi.className = 'breadcrumb-item';
  rootLi.innerHTML = '<a href="#">assets</a>';
  rootLi.onclick = (e) => { e.preventDefault(); navigateTo(''); };
  breadcrumbList.appendChild(rootLi);
  parts.forEach((part, idx) => {
    accum += (accum ? '/' : '') + part;
    const li = document.createElement('li');
    li.className = 'breadcrumb-item' + (idx === parts.length - 1 ? ' active' : '');
    if (idx === parts.length - 1) {
      li.textContent = part;
    } else {
      li.innerHTML = `<a href="#">${part}</a>`;
      li.onclick = (e) => { e.preventDefault(); navigateTo(accum); };
    }
    breadcrumbList.appendChild(li);
  });
}

function navigateTo(path) {
  currentPath = path;
  fetchFiles(path);
}

function deleteFile(assetPath) {
  // assetPath: /assets/123/file.txt or /assets/123
  const match = assetPath.match(/\/assets\/(\d+)/);
  if (!match) return alert('Can only delete numbered asset folders.');
  const n = match[1];
  if (!confirm(`Delete asset folder ${n}? This cannot be undone.`)) return;
  axios.get(`/api/delete_asset/${n}`)
    .then(() => fetchFiles(currentPath))
    .catch(err => alert('Delete failed: ' + (err.response?.data?.error || err.message)));
}

deleteAllBtn.onclick = function() {
  if (!confirm('Delete ALL asset folders? This cannot be undone.')) return;
  axios.get('/api/delete_asset/all')
    .then(() => fetchFiles(currentPath))
    .catch(err => alert('Delete all failed: ' + (err.response?.data?.error || err.message)));
};

refreshBtn.onclick = function() {
  fetchFiles(currentPath);
};

// Initial load
fetchFiles();
