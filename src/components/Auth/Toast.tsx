import React, { useState, useCallback, useEffect, createContext, useContext } from "react";

type ToastVariant = "success" | "error" | "info";

interface Toast {
  id: number;
  message: string;
  variant: ToastVariant;
}

interface ToastContextValue {
  showToast: (message: string, variant?: ToastVariant) => void;
}

const ToastContext = createContext<ToastContextValue>({
  showToast: () => {},
});

let nextId = 0;
const AUTO_DISMISS_MS = 5000;

export function useToast(): ToastContextValue {
  return useContext(ToastContext);
}

export function ToastProvider({
  children,
}: {
  children: React.ReactNode;
}): React.JSX.Element {
  const [toasts, setToasts] = useState<Toast[]>([]);

  const showToast = useCallback((message: string, variant: ToastVariant = "info") => {
    const id = nextId++;
    setToasts((prev) => [...prev, { id, message, variant }]);
  }, []);

  const removeToast = useCallback((id: number) => {
    setToasts((prev) => prev.filter((t) => t.id !== id));
  }, []);

  return (
    <ToastContext.Provider value={{ showToast }}>
      {children}
      <ToastContainer toasts={toasts} onRemove={removeToast} />
    </ToastContext.Provider>
  );
}

function ToastContainer({
  toasts,
  onRemove,
}: {
  toasts: Toast[];
  onRemove: (id: number) => void;
}): React.JSX.Element | null {
  if (toasts.length === 0) return null;

  return (
    <div className="auth-toast-container" aria-live="polite" role="status">
      {toasts.map((toast) => (
        <ToastItem key={toast.id} toast={toast} onRemove={onRemove} />
      ))}
    </div>
  );
}

function ToastItem({
  toast,
  onRemove,
}: {
  toast: Toast;
  onRemove: (id: number) => void;
}): React.JSX.Element {
  useEffect(() => {
    const timer = setTimeout(() => onRemove(toast.id), AUTO_DISMISS_MS);
    return () => clearTimeout(timer);
  }, [toast.id, onRemove]);

  return (
    <div className={`auth-toast auth-toast--${toast.variant}`} role="alert">
      <span>{toast.message}</span>
      <button
        className="auth-toast__close"
        onClick={() => onRemove(toast.id)}
        aria-label="Dismiss notification"
      >
        &times;
      </button>
    </div>
  );
}
