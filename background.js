// With no default_popup in the manifest, clicking the toolbar icon fires this
// instead — open the simulator as a full page.
chrome.action.onClicked.addListener(() => {
  chrome.tabs.create({ url: chrome.runtime.getURL('index.html') });
});
