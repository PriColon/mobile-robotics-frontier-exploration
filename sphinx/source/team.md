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

## Live Repository Stats

<div id="github-stats" style="margin: 20px 0;">
  <p>Loading stats...</p>
</div>

<script>
async function fetchGitHubStats() {
    const repo = 'PriColon/mobile-robotics-frontier-exploration';
    const members = ['PriColon', 'rmane2', 'Mkondamu'];
    const token = '';  // leave empty for public repos

    const headers = { 'Accept': 'application/vnd.github.v3+json' };

    try {
        // Fetch repo info
        const repoRes = await fetch(`https://api.github.com/repos/${repo}`, { headers });
        const repoData = await repoRes.json();

        // Fetch commits
        const commitsRes = await fetch(`https://api.github.com/repos/${repo}/commits?per_page=100`, { headers });
        const commits = await commitsRes.json();

        // Fetch pull requests
        const prsRes = await fetch(`https://api.github.com/repos/${repo}/pulls?state=all&per_page=100`, { headers });
        const prs = await prsRes.json();

        // Count commits per author
        const commitCounts = {};
        members.forEach(m => commitCounts[m.toLowerCase()] = 0);
        commits.forEach(c => {
            if (c.author && c.author.login) {
                const login = c.author.login.toLowerCase();
                if (commitCounts[login] !== undefined) {
                    commitCounts[login]++;
                }
            }
        });

        // Count PRs per author
        const prCounts = {};
        members.forEach(m => prCounts[m.toLowerCase()] = 0);
        prs.forEach(pr => {
            if (pr.user && pr.user.login) {
                const login = pr.user.login.toLowerCase();
                if (prCounts[login] !== undefined) {
                    prCounts[login]++;
                }
            }
        });

        // Build HTML
        const html = `
        <table style="width:100%; border-collapse: collapse;">
            <thead>
                <tr style="background:#2980b9; color:white;">
                    <th style="padding:10px; text-align:left;">Member</th>
                    <th style="padding:10px; text-align:center;">Commits</th>
                    <th style="padding:10px; text-align:center;">Pull Requests</th>
                    <th style="padding:10px; text-align:center;">Commit Bar</th>
                </tr>
            </thead>
            <tbody>
                ${[
                    ['Princess Colon', 'PriColon'],
                    ['Rohit Mane',     'rmane2'],
                    ['Manjunath Kondamu', 'Mkondamu']
                ].map(([name, login], i) => {
                    const c = commitCounts[login.toLowerCase()] || 0;
                    const p = prCounts[login.toLowerCase()] || 0;
                    const maxC = Math.max(...Object.values(commitCounts), 1);
                    const pct = Math.round((c / maxC) * 100);
                    const bg = i % 2 === 0 ? '#f5f5f5' : '#ffffff';
                    return `
                    <tr style="background:${bg};">
                        <td style="padding:10px; font-weight:bold;">${name}</td>
                        <td style="padding:10px; text-align:center;">${c}</td>
                        <td style="padding:10px; text-align:center;">${p}</td>
                        <td style="padding:10px;">
                            <div style="background:#e0e0e0; border-radius:4px; height:16px;">
                                <div style="background:#2980b9; width:${pct}%; height:16px; border-radius:4px;"></div>
                            </div>
                        </td>
                    </tr>`;
                }).join('')}
            </tbody>
        </table>
        <p style="font-size:0.85em; color:#888; margin-top:8px;">
            Total commits: ${commits.length} · 
            Total PRs: ${prs.length} · 
            Last updated: ${new Date().toLocaleString()}
        </p>`;

        document.getElementById('github-stats').innerHTML = html;

    } catch (err) {
        document.getElementById('github-stats').innerHTML = 
            '<p>Could not load stats — check network or GitHub API rate limit.</p>';
    }
}

fetchGitHubStats();
</script>

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
