import React, { useState, useCallback } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useAuth } from "@site/src/contexts/AuthContext";
import { useToast } from "@site/src/components/Auth/Toast";

const SOFTWARE_OPTIONS = [
  "Python",
  "ROS 2",
  "C++",
  "JavaScript",
  "MATLAB",
  "Bash/Shell",
];

const HARDWARE_OPTIONS = [
  "Jetson Orin",
  "Desktop Workstation",
  "Laptop",
  "Raspberry Pi",
  "Cloud/VM",
];

interface Props {
  initialSoftware?: string[];
  initialHardware?: string[];
  initialSoftwareOther?: string;
  initialHardwareOther?: string;
  onSave?: () => void;
  onSkip?: () => void;
  showSkip?: boolean;
}

export default function BackgroundForm({
  initialSoftware = [],
  initialHardware = [],
  initialSoftwareOther = "",
  initialHardwareOther = "",
  onSave,
  onSkip,
  showSkip = true,
}: Props): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const authApiUrl = (siteConfig.customFields?.authApiUrl as string) || "http://localhost:3005";
  const { refetch } = useAuth();
  const { showToast } = useToast();
  const [software, setSoftware] = useState<string[]>(initialSoftware);
  const [hardware, setHardware] = useState<string[]>(initialHardware);
  const [softwareOther, setSoftwareOther] = useState(initialSoftwareOther);
  const [hardwareOther, setHardwareOther] = useState(initialHardwareOther);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const toggleOption = useCallback(
    (list: string[], setList: React.Dispatch<React.SetStateAction<string[]>>, item: string) => {
      setList(list.includes(item) ? list.filter((i) => i !== item) : [...list, item]);
    },
    []
  );

  const handleSubmit = useCallback(
    async (e: React.FormEvent) => {
      e.preventDefault();
      setIsSubmitting(true);

      try {
        const response = await fetch(
          `${authApiUrl}/api/auth/custom/update-background`,
          {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            credentials: "include",
            body: JSON.stringify({
              softwareBackground: software,
              hardwareBackground: hardware,
              softwareOther: softwareOther.trim() || undefined,
              hardwareOther: hardwareOther.trim() || undefined,
            }),
          }
        );

        if (!response.ok) {
          const data = await response.json().catch(() => null);
          throw new Error(data?.error || "Failed to save background");
        }

        showToast("Background saved!", "success");
        refetch();
        onSave?.();
      } catch (err) {
        showToast(
          err instanceof Error ? err.message : "Failed to save background",
          "error"
        );
      } finally {
        setIsSubmitting(false);
      }
    },
    [software, hardware, softwareOther, hardwareOther, showToast, refetch, onSave]
  );

  return (
    <form className="background-form" onSubmit={handleSubmit}>
      <div className="background-form__section">
        <h3>Software Experience</h3>
        <p>Select all that apply:</p>
        <div className="background-form__options">
          {SOFTWARE_OPTIONS.map((opt) => (
            <label key={opt} className="background-form__checkbox">
              <input
                type="checkbox"
                checked={software.includes(opt)}
                onChange={() => toggleOption(software, setSoftware, opt)}
              />
              {opt}
            </label>
          ))}
        </div>
        <div className="background-form__other">
          <label htmlFor="software-other">Other:</label>
          <input
            id="software-other"
            type="text"
            maxLength={255}
            value={softwareOther}
            onChange={(e) => setSoftwareOther(e.target.value)}
            placeholder="Other software experience"
          />
        </div>
      </div>

      <div className="background-form__section">
        <h3>Hardware Setup</h3>
        <p>Select all that apply:</p>
        <div className="background-form__options">
          {HARDWARE_OPTIONS.map((opt) => (
            <label key={opt} className="background-form__checkbox">
              <input
                type="checkbox"
                checked={hardware.includes(opt)}
                onChange={() => toggleOption(hardware, setHardware, opt)}
              />
              {opt}
            </label>
          ))}
        </div>
        <div className="background-form__other">
          <label htmlFor="hardware-other">Other:</label>
          <input
            id="hardware-other"
            type="text"
            maxLength={255}
            value={hardwareOther}
            onChange={(e) => setHardwareOther(e.target.value)}
            placeholder="Other hardware"
          />
        </div>
      </div>

      <div className="background-form__actions">
        <button
          type="submit"
          className="background-form__submit"
          disabled={isSubmitting}
        >
          {isSubmitting ? "Saving..." : "Save Background"}
        </button>
        {showSkip && onSkip && (
          <button
            type="button"
            className="background-form__skip"
            onClick={onSkip}
          >
            Skip for now
          </button>
        )}
      </div>
    </form>
  );
}
