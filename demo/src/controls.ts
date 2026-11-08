// The control builders every page uses (slider, dropdown, checkbox, button, readout box), so the
// pages share one look; styles/main.css styles them. A slider sits in a wrapper that takes touch
// input in its place (src/slider-touch.ts), so a finger scrolling the page past it does not move it.

import { attachTouchDrag } from './slider-touch.ts';

export function createSlider(
  label: string, min: number, max: number, value: number, step: number,
  onChange: (val: number) => void,
): HTMLElement {
  const group = document.createElement('div');
  group.className = 'control-group';
  const lbl = document.createElement('label');
  const labelText = document.createTextNode(label);
  const valSpan = document.createElement('span');
  valSpan.className = 'slider-value';
  valSpan.textContent = String(value);
  lbl.appendChild(labelText);
  lbl.appendChild(valSpan);

  const input = document.createElement('input');
  input.type = 'range';
  input.min = String(min);
  input.max = String(max);
  input.step = String(step);
  input.value = String(value);
  input.addEventListener('input', () => {
    const v = parseFloat(input.value);
    valSpan.textContent = String(v);
    onChange(v);
  });

  const hold = document.createElement('div');
  hold.className = 'slider-hold';
  hold.appendChild(input);
  attachTouchDrag(hold, input);

  group.appendChild(lbl);
  group.appendChild(hold);
  return group;
}

export function createDropdown(
  label: string,
  options: { value: string; text: string }[],
  selectedValue: string,
  onChange: (val: string) => void,
): HTMLElement {
  const group = document.createElement('div');
  group.className = 'control-group';
  const lbl = document.createElement('label');
  lbl.textContent = label;

  const select = document.createElement('select');
  for (const opt of options) {
    const o = document.createElement('option');
    o.value = opt.value;
    o.textContent = opt.text;
    select.appendChild(o);
  }
  select.value = selectedValue;
  select.addEventListener('change', () => onChange(select.value));

  group.appendChild(lbl);
  group.appendChild(select);
  return group;
}

export function createCheckbox(
  label: string, checked: boolean,
  onChange: (val: boolean) => void,
): HTMLElement {
  const wrapper = document.createElement('label');
  wrapper.className = 'control-checkbox';
  const input = document.createElement('input');
  input.type = 'checkbox';
  input.checked = checked;
  input.addEventListener('change', () => onChange(input.checked));
  const span = document.createElement('span');
  span.textContent = label;
  wrapper.appendChild(input);
  wrapper.appendChild(span);
  return wrapper;
}

export function createButton(label: string, onClick: () => void): HTMLButtonElement {
  const btn = document.createElement('button');
  btn.className = 'control-btn';
  btn.textContent = label;
  btn.addEventListener('click', onClick);
  return btn;
}

