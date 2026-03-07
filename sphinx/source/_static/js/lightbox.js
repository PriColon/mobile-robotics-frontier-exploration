document.addEventListener("DOMContentLoaded", function() {
    // Find the architecture image and add zoom class
    document.querySelectorAll('img').forEach(function(img) {
        if (img.src.includes('architecture')) {
            img.classList.add('architecture-img');
            img.title = 'Click to zoom';

            // Create lightbox
            var overlay = document.createElement('div');
            overlay.className = 'lightbox-overlay';
            var bigImg = document.createElement('img');
            bigImg.src = img.src;
            overlay.appendChild(bigImg);
            document.body.appendChild(overlay);

            // Click image → open lightbox
            img.addEventListener('click', function() {
                overlay.classList.add('active');
            });

            // Click overlay → close lightbox
            overlay.addEventListener('click', function() {
                overlay.classList.remove('active');
            });
        }
    });
});
