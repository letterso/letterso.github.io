// Search Modal JavaScript
(function() {
    'use strict';

    // Global search data
    let allPosts = [];
    let searchModal = null;
    let searchBackdrop = null;
    let searchInput = null;
    let searchResults = null;

    // Initialize search functionality
    function initSearch() {
        // Create modal HTML
        createSearchModal();
        
        // Load posts data
        loadPostsData();
        
        // Setup event listeners
        setupEventListeners();
    }

    // Create search modal HTML
    function createSearchModal() {
        // Create backdrop
        searchBackdrop = document.createElement('div');
        searchBackdrop.className = 'search-modal-backdrop';
        searchBackdrop.id = 'search-modal-backdrop';
        document.body.appendChild(searchBackdrop);

        // Create modal
        searchModal = document.createElement('div');
        searchModal.className = 'search-modal';
        searchModal.id = 'search-modal';
        searchModal.innerHTML = `
            <div class="search-modal-header">
                <h2>搜索文章</h2>
                <button class="search-modal-close" id="search-modal-close">
                    <i class="fa fa-times"></i>
                </button>
            </div>
            <div class="search-input-container">
                <div class="search-input-wrapper">
                    <input 
                        type="text" 
                        class="search-input" 
                        id="search-modal-input" 
                        placeholder="输入关键词搜索..." 
                        autocomplete="off"
                    >
                    <span class="search-input-icon">
                        <i class="fa fa-search"></i>
                    </span>
                </div>
            </div>
            <div class="search-stats" id="search-stats"></div>
            <div class="search-results" id="search-results">
                <div class="search-no-results">请输入关键词进行搜索</div>
            </div>
        `;
        document.body.appendChild(searchModal);

        // Get element references
        searchInput = document.getElementById('search-modal-input');
        searchResults = document.getElementById('search-results');
    }

    // Load posts data from algolia.json or local index
    function loadPostsData() {
        // Try to load from algolia.json first
        fetch(getBaseUrl() + '/algolia.json')
            .then(response => response.json())
            .then(data => {
                allPosts = data;
            })
            .catch(error => {
                console.warn('Failed to load algolia.json:', error);
                // Fallback: extract posts from page if available
                extractPostsFromPage();
            });
    }

    // Extract posts from page data if available
    function extractPostsFromPage() {
        // This is a fallback method if algolia.json is not available
        // It would extract post information from the page
        allPosts = window.postsData || [];
    }

    // Get base URL for the site
    function getBaseUrl() {
        if (typeof baseURL !== 'undefined') {
            return baseURL.replace(/\/$/, '');
        }
        return '';
    }

    // Setup event listeners
    function setupEventListeners() {
        // Search icon click
        const searchIcon = document.querySelector('a[href*="search"]');
        if (searchIcon) {
            searchIcon.addEventListener('click', openSearchModal);
        }

        // Also add keyboard shortcut (Ctrl+K or Cmd+K)
        document.addEventListener('keydown', handleKeyPress);

        // Modal close button
        const closeBtn = document.getElementById('search-modal-close');
        if (closeBtn) {
            closeBtn.addEventListener('click', closeSearchModal);
        }

        // Backdrop click
        if (searchBackdrop) {
            searchBackdrop.addEventListener('click', closeSearchModal);
        }

        // Search input
        if (searchInput) {
            searchInput.addEventListener('input', handleSearch);
            searchInput.addEventListener('keydown', handleInputKeydown);
        }

        // Prevent backdrop click from closing when clicking inside modal
        if (searchModal) {
            searchModal.addEventListener('click', e => e.stopPropagation());
        }

        // Prevent closing on Escape initially, let it be handled by handleKeyPress
        document.addEventListener('keydown', e => {
            if (e.key === 'Escape' && searchModal && searchModal.classList.contains('active')) {
                closeSearchModal();
            }
        });
    }

    // Handle keyboard press
    function handleKeyPress(e) {
        // Ctrl+K or Cmd+K to open search
        if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
            e.preventDefault();
            openSearchModal();
        }
    }

    // Handle search input keydown
    function handleInputKeydown(e) {
        if (e.key === 'Escape') {
            closeSearchModal();
        }
    }

    // Open search modal
    function openSearchModal() {
        if (!searchModal || !searchBackdrop) return;
        
        searchModal.classList.add('active');
        searchBackdrop.classList.add('active');
        searchInput?.focus();
    }

    // Close search modal
    function closeSearchModal() {
        if (!searchModal || !searchBackdrop) return;
        
        searchModal.classList.remove('active');
        searchBackdrop.classList.remove('active');
        if (searchInput) searchInput.value = '';
        if (searchResults) {
            searchResults.innerHTML = '<div class="search-no-results">请输入关键词进行搜索</div>';
        }
    }

    // Perform search
    function handleSearch(e) {
        const query = e.target.value.trim().toLowerCase();
        
        if (!query) {
            searchResults.innerHTML = '<div class="search-no-results">请输入关键词进行搜索</div>';
            document.getElementById('search-stats').innerHTML = '';
            return;
        }

        const results = performSearch(query);
        displayResults(results, query);
    }

    // Perform the actual search
    function performSearch(query) {
        if (!allPosts || allPosts.length === 0) {
            return [];
        }

        const queryTerms = query.split(/\s+/).filter(term => term.length > 0);
        
        return allPosts.filter(post => {
            // Search in title, content, tags, and categories
            const searchableText = [
                post.title || '',
                post.html || '',
                post.content || '',
                (post.tags || []).join(' '),
                (post.categories || []).join(' ')
            ].join(' ').toLowerCase();

            // All query terms must match
            return queryTerms.every(term => searchableText.includes(term));
        }).sort((a, b) => {
            // Score based on title match (higher priority)
            const aTitle = (a.title || '').toLowerCase();
            const bTitle = (b.title || '').toLowerCase();
            
            const aTitleMatch = aTitle.includes(query) ? 2 : 0;
            const bTitleMatch = bTitle.includes(query) ? 2 : 0;
            
            if (aTitleMatch !== bTitleMatch) {
                return bTitleMatch - aTitleMatch;
            }
            
            // Then sort by date (newer first)
            return (b.date || 0) - (a.date || 0);
        }).slice(0, 50); // Limit to 50 results
    }

    // Display search results
    function displayResults(results, query) {
        const statsEl = document.getElementById('search-stats');
        
        if (results.length === 0) {
            statsEl.innerHTML = '';
            searchResults.innerHTML = '<div class="search-no-results">未找到相关文章</div>';
            return;
        }

        statsEl.innerHTML = `找到 ${results.length} 个结果`;
        
        let html = '';
        results.forEach(post => {
            const date = formatDate(post.date);
            const category = (post.categories && post.categories[0]) || '';
            const snippet = extractSnippet(post.content || post.html || '', query, 100);
            const highlightedTitle = highlightQuery(post.title || '', query);
            
            html += `
                <div class="search-result-item" data-url="${post.url || post.permalink || '#'}">
                    ${category ? `<div class="search-result-category">${escapeHtml(category)}</div>` : ''}
                    <div class="search-result-title">${highlightedTitle}</div>
                    ${snippet ? `<div class="search-result-snippet">${snippet}</div>` : ''}
                    ${date ? `<div class="search-result-date">${date}</div>` : ''}
                </div>
            `;
        });
        
        searchResults.innerHTML = html;
        
        // Add click handlers to results
        document.querySelectorAll('.search-result-item').forEach(item => {
            item.addEventListener('click', () => {
                const url = item.dataset.url;
                if (url && url !== '#') {
                    window.location.href = url;
                }
            });
            item.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') {
                    const url = item.dataset.url;
                    if (url && url !== '#') {
                        window.location.href = url;
                    }
                }
            });
        });
    }

    // Format date
    function formatDate(timestamp) {
        if (!timestamp) return '';
        
        try {
            const date = new Date(timestamp * 1000);
            return date.toLocaleDateString('zh-CN', {
                year: 'numeric',
                month: 'short',
                day: 'numeric'
            });
        } catch (e) {
            return '';
        }
    }

    // Extract snippet from content
    function extractSnippet(content, query, maxLength) {
        if (!content) return '';
        
        // Remove HTML tags
        const text = content.replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim();
        
        const index = text.toLowerCase().indexOf(query.toLowerCase());
        if (index === -1) {
            return text.substring(0, maxLength) + (text.length > maxLength ? '...' : '');
        }

        const start = Math.max(0, index - 30);
        const end = Math.min(text.length, index + query.length + 70);
        
        let snippet = text.substring(start, end);
        if (start > 0) snippet = '...' + snippet;
        if (end < text.length) snippet += '...';
        
        return highlightQuery(snippet, query);
    }

    // Highlight query terms in text
    function highlightQuery(text, query) {
        if (!text || !query) return escapeHtml(text);
        
        const regex = new RegExp(`(${escapeRegex(query)})`, 'gi');
        return escapeHtml(text).replace(regex, '<em>$1</em>');
    }

    // Escape HTML special characters
    function escapeHtml(text) {
        if (!text) return '';
        const map = {
            '&': '&amp;',
            '<': '&lt;',
            '>': '&gt;',
            '"': '&quot;',
            "'": '&#039;'
        };
        return text.replace(/[&<>"']/g, char => map[char]);
    }

    // Escape regex special characters
    function escapeRegex(str) {
        return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    }

    // Initialize when DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initSearch);
    } else {
        initSearch();
    }

    // Expose functions globally for testing
    window.searchModalAPI = {
        open: openSearchModal,
        close: closeSearchModal
    };
})();
