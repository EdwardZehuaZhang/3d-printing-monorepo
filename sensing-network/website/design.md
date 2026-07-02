# SenseKit Website - Design Style Guide

## Stack

- **Framework:** Next.js 16 + React 19 + TypeScript
- **Styling:** Tailwind CSS 4 with CSS custom properties
- **Animation:** Framer Motion 12
- **3D:** Three.js + react-three-fiber + react-three-drei
- **Icons:** Lucide React
- **Font:** IBM Plex Mono (weights 300-700)

---

## Color Palette

All colors defined as CSS custom properties in `globals.css`.

### Brand

| Token               | Value     | Usage                  |
| -------------------- | --------- | ---------------------- |
| `--color-primary`       | `#1245A8` | Main brand, CTAs       |
| `--color-primary-light` | `#2a6bdb` | Hover states           |
| `--color-primary-dark`  | `#0e3580` | Active/pressed states  |
| `--color-secondary`     | `#2563eb` | Secondary actions      |
| `--color-secondary-light` | `#60a5fa` | Light secondary accents |
| `--color-secondary-dark` | `#1d4ed8` | Dark secondary accents |

### Semantic

| Token            | Value     | Usage                |
| ---------------- | --------- | -------------------- |
| `--color-success` | `#16a34a` | Confirmations, checks |
| `--color-warning` | `#d97706` | Warnings, caution     |
| `--color-danger`  | `#dc2626` | Errors, destructive   |

### Surfaces

| Token                  | Value     | Usage                    |
| ---------------------- | --------- | ------------------------ |
| `--color-surface`       | `#ffffff` | Page backgrounds         |
| `--color-surface-raised` | `#f9fafb` | Elevated cards           |
| `--color-surface-inset` | `#f3f4f6` | Recessed areas, code blocks |

### Text

| Token                  | Value     | Usage             |
| ---------------------- | --------- | ----------------- |
| `--color-text-primary`  | `#111827` | Headlines, body   |
| `--color-text-secondary` | `#6b7280` | Descriptions      |
| `--color-text-tertiary` | `#9ca3af` | Disabled, muted   |

### Borders

| Token                 | Value     | Usage            |
| --------------------- | --------- | ---------------- |
| `--color-border`       | `#e5e7eb` | Standard borders |
| `--color-border-strong` | `#d1d5db` | Emphasized borders |

---

## Typography

The entire site uses **IBM Plex Mono** -- a monospace font chosen for a technical/maker aesthetic.

| Element       | Classes                                        | Notes                      |
| ------------- | ---------------------------------------------- | -------------------------- |
| H1            | `text-4xl md:text-6xl font-bold tracking-tight` | Hero headlines             |
| H2            | `text-3xl font-bold tracking-tight`             | Section headings           |
| H3            | `text-xl font-bold` or `text-lg font-bold`     | Subsections                |
| Label/Tag     | `text-xs font-semibold uppercase tracking-widest` | Small caps badges, labels |
| Body          | `text-sm` or `text-base`, `text-text-secondary`  | Readable secondary color  |
| Code inline   | `bg-surface-inset px-1.5 py-0.5 border border-border rounded` | Inline code snippets |
| Code block    | `bg-surface-inset border border-border rounded-xl p-4`       | Multi-line code      |

**Key traits:** `tracking-tight` on headlines, `tracking-widest` on uppercase labels, `leading-relaxed` on body paragraphs, antialiased rendering.

---

## Spacing & Layout

### Container

- Max width: `max-w-6xl` (1024px) centered with `mx-auto`
- Horizontal padding: `px-6` (consistent at all breakpoints)
- Section vertical padding: `py-16` / `md:py-24`

### Grid Patterns

| Layout        | Classes                    | Usage              |
| ------------- | -------------------------- | ------------------ |
| 2-col         | `md:grid-cols-2 gap-8`    | Hero, alternating  |
| 3-col         | `md:grid-cols-3 gap-6`    | Feature cards      |
| 4-col         | `sm:grid-cols-2 lg:grid-cols-4 gap-6` | Use cases |

### Responsive Approach

Mobile-first. Base styles are single column; `md:` (768px) adds grid layouts. Key breakpoints: `sm:` (640px), `md:` (768px), `lg:` (1024px).

---

## Border & Radius

| Pattern           | Value          | Usage                     |
| ----------------- | -------------- | ------------------------- |
| Standard border   | `border-2`     | Cards, buttons, nav items |
| Section divider   | `border-b-2 border-border` | Between sections |
| Small radius      | `rounded-lg`   | Nav links, small elements |
| Medium radius     | `rounded-xl`   | Buttons, code blocks      |
| Large radius      | `rounded-2xl`  | Cards                     |
| Pill              | `rounded-full` | Badges, tags              |

---

## Components

### Buttons

```
Primary:   border-2 border-primary bg-primary text-white rounded-xl px-6 py-3
           text-sm font-bold uppercase tracking-widest
           hover:-translate-y-0.5 hover:shadow-lg hover:shadow-primary/20

Secondary: border-2 border-border text-text-primary rounded-xl px-6 py-3
           text-sm font-bold uppercase tracking-widest
           hover:-translate-y-0.5 hover:shadow-lg
```

### Cards

```
border-2 border-border rounded-2xl p-6 bg-surface
border-t-[3px] border-t-{color}              // colored top accent
hover:-translate-y-1 hover:shadow-lg          // lift on hover
transition-all
```

### Badges / Tags

```
rounded-full border-2 px-4 py-1.5
text-xs font-semibold uppercase tracking-widest
```

Solid variant: `bg-primary text-white rounded-full px-3 py-1 text-[11px]`

### Navigation

- Sticky top, `z-50`, `bg-surface/95 backdrop-blur-sm`
- `border-b-2 border-border` separator
- Active link: `bg-surface-inset text-primary`
- Mobile: hamburger menu with animated line transforms

---

## Animation

### Framer Motion - Fade Up (primary pattern)

```ts
const fadeUp = {
  hidden: { opacity: 0, y: 24 },
  visible: (i: number) => ({
    opacity: 1,
    y: 0,
    transition: { delay: i * 0.1, duration: 0.5, ease: "easeOut" },
  }),
};
```

- Stagger: 100ms per item
- Duration: 500ms
- Triggered `whileInView` with `once: true`

### Hover Micro-interactions

- Cards: `-translate-y-1` + `shadow-lg`
- Buttons: `-translate-y-0.5` + `shadow-lg`
- All use `transition-all`

### 3D Model

- Auto-rotates on z-axis (`delta * 0.3`)
- Entry: scale `0.95 -> 1`, opacity `0 -> 1`, delay `0.3s`
- Interactive orbit controls

---

## Shadows

- Standard: `shadow-lg`
- Colored: `shadow-primary/20`, `shadow-primary/30`
- Applied on hover, not at rest

---

## Special Effects

- **Grid background:** Subtle CSS linear-gradient grid at `opacity-[0.03]`, `backgroundSize: 32px 32px`
- **Smooth scroll:** `html { scroll-behavior: smooth }`
- **Text selection:** Primary color background with white text
- **Dark CTA sections:** `bg-text-primary` with inverted text colors

---

## Information Hierarchy

| Level     | Color              | Weight         |
| --------- | ------------------ | -------------- |
| Primary   | `text-text-primary`  | `font-bold`    |
| Secondary | `text-text-secondary` | `font-normal`  |
| Tertiary  | `text-text-tertiary` | `font-normal`  |
| Link      | `text-primary`       | hover underline |
| Disabled  | `text-text-tertiary` | `line-through`  |

---

## Design Principles

1. **Technical aesthetic** -- IBM Plex Mono throughout signals precision and a maker ethos
2. **Clean structure** -- 2px borders, generous whitespace, clear hierarchy
3. **Restrained color** -- Deep blue brand palette with semantic accents; no gratuitous color
4. **Subtle motion** -- Fade-up on scroll, gentle lifts on hover; nothing distracting
5. **Mobile-first responsive** -- Single column stacks gracefully, grids appear at `md:`
6. **Consistent tokens** -- All colors via CSS variables; spacing follows Tailwind defaults
