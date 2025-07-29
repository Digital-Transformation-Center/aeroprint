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
    li.className = 'list-group-item d-flex justify-content-between align-items-center';
    if (item.type === 'directory') {
      li.innerHTML = `<span>📁 <a href="#">${item.name}</a></span>`;
      li.querySelector('a').onclick = (e) => {
        e.preventDefault();
        navigateTo(item.path.replace('/api/list_assets/', ''));
      };
    } else {
      li.innerHTML = `<span>📄 <a href="${item.path}" target="_blank">${item.name}</a></span>`;
      const delBtn = document.createElement('button');
      delBtn.className = 'btn btn-danger btn-sm';
      delBtn.textContent = 'Delete';
      delBtn.onclick = () => deleteFile(item.path);
      li.appendChild(delBtn);
    }
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
