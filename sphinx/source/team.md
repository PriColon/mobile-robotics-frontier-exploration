# Team

**Mobile Robotics — ASU Spring 2026**

---

## Members

| Name | Role | GitHub |
|---|---|---|
| Princess Colon | Perception + SLAM (Person A) | [@PriColon](https://github.com/PriColon) |
| Rohit Mane | Custom Algorithm + Classifier (Person B) | [@rmane2](https://github.com/rmane2) |
| Manjunath Kondamu | Navigation + Integration (Person C) | [@Mkondamu](https://github.com/Mkondamu) |

---

## Live Repository Status

<div id="repo-stats" style="margin: 20px 0;">
  <p>Loading repository stats...</p>
</div>

---

## Commit Activity per Member

<div id="github-stats" style="margin: 20px 0;">
  <p>Loading member stats...</p>
</div>

---

## Recent Commits

<div id="commit-table" style="margin: 20px 0;">
  <p>Loading commits...</p>
</div>

---

## Responsibilities

**Princess Colon — Person A**
- slam_toolbox configuration and tuning
- LiDAR scan processing and verification
- OAK-D camera driver setup and calibration
- EKF localization configuration

**Rohit Mane — Person B**
- frontier_explorer_node.py — core algorithm
- semantic_classifier_node.py
- Parameter tuning and unit tests
- Frontier marker visualization

**Manjunath Kondamu — Person C**
- behavior_coordinator_node.py
- explore.launch.py — single launch command
- System integration and safety failsafe
- Documentation site (this site)

---

<script>
const REPO    = 'PriColon/mobile-robotics-frontier-exploration';
const API     = 'https://api.github.com';
const MEMBERS = [
    { name: 'Princess Colon',    login: 'PriColon' },
    { name: 'Rohit Mane',        login: 'rmane2'   },
    { name: 'Manjunath Kondamu', login: 'Mkondamu' },
];

async function fetchJSON(url) {
    const res = await fetch(url, {
        headers: { 'Accept': 'application/vnd.github.v3+json' }
    });
    return res.json();
}

async function loadAll() {
    try {
        const repo     = await fetchJSON(`${API}/repos/${REPO}`);
        const commits  = await fetchJSON(`${API}/repos/${REPO}/commits?per_page=100`);
        const branches = await fetchJSON(`${API}/repos/${REPO}/branches`);
        const prs      = await fetchJSON(`${API}/repos/${REPO}/pulls?state=all&per_page=100`);

        // ── REPO STAT CARDS ───────────────────────────────────────────
        const lastPush = new Date(repo.pushed_at).toLocaleString();
        document.getElementById('repo-stats').innerHTML = `
        <div style="display:flex; gap:16px; flex-wrap:wrap; margin-bottom:12px;">
            ${[
                ['⭐ Stars',          repo.stargazers_count],
                ['🍴 Forks',          repo.forks_count],
                ['👁️ Watchers',       repo.watchers_count],
                ['🌿 Branches',       branches.length],
                ['📝 Total Commits',  commits.length],
                ['🔀 Pull Requests',  prs.length],
            ].map(([label, val]) => `
                <div style="background:#2c3e50; color:white; padding:14px 20px;
                            border-radius:8px; text-align:center; min-width:100px;">
                    <div style="font-size:1.6em; font-weight:bold;">${val}</div>
                    <div style="font-size:0.8em; margin-top:4px; color:#aaa;">${label}</div>
                </div>`
            ).join('')}
        </div>
        <p style="font-size:0.85em; color:#888;">
            🕐 Last push: ${lastPush} &nbsp;|&nbsp;
            📦 Branch: <code>${repo.default_branch}</code> &nbsp;|&nbsp;
            🔓 ${repo.private ? 'Private' : 'Public'}
        </p>`;

        // ── PER-MEMBER STATS ──────────────────────────────────────────
        const commitCounts = {};
        const prCounts     = {};
        MEMBERS.forEach(m => {
            commitCounts[m.login.toLowerCase()] = 0;
            prCounts[m.login.toLowerCase()]     = 0;
        });
        commits.forEach(c => {
            if (c.author && c.author.login) {
                const l = c.author.login.toLowerCase();
                if (commitCounts[l] !== undefined) commitCounts[l]++;
            }
        });
        prs.forEach(pr => {
            if (pr.user && pr.user.login) {
                const l = pr.user.login.toLowerCase();
                if (prCounts[l] !== undefined) prCounts[l]++;
            }
        });

        const maxC = Math.max(...Object.values(commitCounts), 1);
        document.getElementById('github-stats').innerHTML = `
        <table style="width:100%; border-collapse:collapse;">
            <thead>
                <tr style="background:#2c3e50; color:white;">
                    <th style="padding:10px; text-align:left;">Member</th>
                    <th style="padding:10px; text-align:center;">Commits</th>
                    <th style="padding:10px; text-align:center;">Pull Requests</th>
                    <th style="padding:10px; text-align:left;">Commit Bar</th>
                </tr>
            </thead>
            <tbody>
                ${MEMBERS.map(({ name, login }, i) => {
                    const c   = commitCounts[login.toLowerCase()] || 0;
                    const p   = prCounts[login.toLowerCase()]     || 0;
                    const pct = Math.round((c / maxC) * 100);
                    const bg  = i % 2 === 0 ? '#f5f5f5' : '#ffffff';
                    return `
                    <tr style="background:${bg};">
                        <td style="padding:10px; font-weight:bold;">${name}</td>
                        <td style="padding:10px; text-align:center;">${c}</td>
                        <td style="padding:10px; text-align:center;">${p}</td>
                        <td style="padding:10px;">
                            <div style="background:#e0e0e0; border-radius:4px; height:16px;">
                                <div style="background:#2980b9; width:${pct}%;
                                            height:16px; border-radius:4px;"></div>
                            </div>
                        </td>
                    </tr>`;
                }).join('')}
            </tbody>
        </table>`;

        // ── COMMIT TABLE (5 rows visible, scrollable) ─────────────────
        const rows = commits.slice(0, 50).map(c => {
            const sha7    = c.sha.substring(0, 7);
            const author  = c.author ? c.author.login : (c.commit.author.name || 'unknown');
            const message = c.commit.message.split('\n')[0].substring(0, 55);
            const date    = new Date(c.commit.author.date).toLocaleDateString();
            const url     = c.html_url;
            const zipUrl  = `https://github.com/${REPO}/archive/${c.sha}.zip`;
            const tarUrl  = `https://github.com/${REPO}/archive/${c.sha}.tar.gz`;
            return `
            <tr>
                <td style="padding:8px; font-family:monospace;">
                    <a href="${url}" target="_blank"
                       style="color:#2980b9; text-decoration:none;">${sha7}</a>
                </td>
                <td style="padding:8px;">${author}</td>
                <td style="padding:8px;">${date}</td>
                <td style="padding:8px; max-width:260px;
                           white-space:nowrap; overflow:hidden;
                           text-overflow:ellipsis;">${message}</td>
                <td style="padding:8px; white-space:nowrap;">
                    <a href="${zipUrl}"
                       style="background:#27ae60; color:white; padding:3px 8px;
                              border-radius:4px; text-decoration:none;
                              font-size:0.8em; margin-right:4px;">⬇ zip</a>
                    <a href="${tarUrl}"
                       style="background:#2980b9; color:white; padding:3px 8px;
                              border-radius:4px; text-decoration:none;
                              font-size:0.8em;">⬇ tar</a>
                </td>
            </tr>`;
        }).join('');

        document.getElementById('commit-table').innerHTML = `
        <div style="overflow-y:auto; max-height:220px; border:1px solid #ddd; border-radius:4px;">
            <table style="width:100%; border-collapse:collapse; font-size:0.88em;">
                <thead style="position:sticky; top:0; z-index:1;">
                    <tr style="background:#2c3e50; color:white;">
                        <th style="padding:10px; text-align:left;">Commit</th>
                        <th style="padding:10px; text-align:left;">Author</th>
                        <th style="padding:10px; text-align:left;">Date</th>
                        <th style="padding:10px; text-align:left;">Message</th>
                        <th style="padding:10px; text-align:left;">Download</th>
                    </tr>
                </thead>
                <tbody>${rows}</tbody>
            </table>
        </div>
        <p style="font-size:0.8em; color:#888; margin-top:6px;">
            Showing up to 50 commits · Scroll to see more ·
            <a href="https://github.com/${REPO}/commits/main" target="_blank">
                View all on GitHub
            </a>
        </p>`;

    } catch(err) {
        document.getElementById('repo-stats').innerHTML =
            '<p style="color:red;">Could not load — check network or GitHub API rate limit.</p>';
    }
}

loadAll();
</script>
