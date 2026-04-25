/**
 * Guard against undefined event.key during browser autocomplete/autofill.
 * The search-local plugin's matchesKeymap() crashes on event.key.toLowerCase()
 * when key is undefined. A capturing listener stops these events early.
 */
if (typeof document !== 'undefined') {
  document.addEventListener(
    'keydown',
    (e) => {
      if (e.key === undefined) {
        e.stopImmediatePropagation();
      }
    },
    true, // capture phase — runs before the plugin's bubble-phase handler
  );
}

export {};
