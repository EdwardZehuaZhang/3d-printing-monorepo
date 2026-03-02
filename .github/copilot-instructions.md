# AndYou Monorepo - AI Coding Assistant Instructions

## Project Architecture

This is a **healthcare monorepo** with three main components sharing a unified PostgreSQL database:

- **`andyou-chatbot/`** - AI medical intake chatbot (Next.js 16, Drizzle ORM, Vercel AI SDK)
- **`admin-website/`** - Medical CMS with dual portals (Next.js 16, Supabase client)
- **`database/`** - Centralized schemas, migrations, and AI agent architecture docs

### Database Strategy

**Single database, dual access patterns:**
- `andyou-chatbot` uses **Drizzle ORM** for agentic workflows
- `admin-website` uses **Supabase client** for admin/doctor operations
- Migrations live in `database/migrations/` and are applied via both Drizzle and Supabase

**Critical:** When changing schema:
1. Update SQL in `database/migrations/`
2. Regenerate types: `pnpm db:generate` (chatbot) and Supabase type gen (admin)
3. Update both `andyou-chatbot/lib/db/schema.ts` and `admin-website/types/database.ts`

## Type Safety Requirements

**Never use `any` - this is enforced strictly.** Follow `.cursor/rules/typescript-type-safety.mdc`:

```typescript
// ✅ Always use generated database types
import type { Database } from '@/types/database'
type Patient = Database['public']['Tables']['patients']['Row']

// ✅ Zod schemas must match database types
const schema = z.object({...}) satisfies z.ZodType<PatientInsert>

// ✅ Handle nulls explicitly
const patient: Patient | null = await getPatient(id)
if (!patient) return { error: 'Not found' }
```

After schema changes:
- **Admin website:** `npx supabase gen types typescript --local > types/database.ts`
- **Chatbot:** `pnpm db:generate` then `pnpm db:migrate`

## Authentication & Security

### Admin Website: RLS-First Security

**Primary boundary:** Supabase Row Level Security policies in `database/schema.sql`  
**Secondary UX layer:** Application RBAC helpers in `lib/rbac.ts`

```typescript
// ✅ Server actions pattern (see app/actions/users.ts)
export async function updateUser(id: string, data: Update) {
  await requireAdmin() // UX - provides clear error message
  const supabase = await createClient() // Authenticated client
  
  // RLS enforces actual security even if requireAdmin() is bypassed
  const { error } = await supabase.from('users').update(data).eq('id', id)
  
  revalidatePath('/dashboard/users')
  return { success: !error }
}

// ❌ Never use service role to bypass RLS except in lib/supabase/admin.ts
// ❌ Client-side role checks are UX only, not security
```

**Roles:** `admin` (full access) | `doctor` (clinical) | `staff` (read-only)  
**Middleware:** `/dashboard/*` for staff/admin, `/doctor/*` for doctors (see `lib/supabase/middleware.ts`)

### Chatbot: NextAuth Credentials

Uses bcrypt-hashed passwords with session management (see `app/(auth)/auth.ts`). Form validation with Zod before auth.

## Development Workflows

### Running Locally

```powershell
# Admin website
cd admin-website; npm install; npm run dev

# Chatbot (uses pnpm)
cd andyou-chatbot; pnpm install; pnpm dev
```

### Database Migrations

```powershell
# Apply migrations (chatbot - runs on build)
cd andyou-chatbot
pnpm db:migrate  # tsx lib/db/migrate.ts

# Generate new migration
pnpm db:generate  # after changing lib/db/schema.ts

# Supabase migrations (admin website)
# Apply via Supabase dashboard or supabase CLI
```

### Building for Production

```powershell
# Admin website (outputs standalone server)
npm run build  # next build with output: "standalone"

# Chatbot (includes migration step)
pnpm build  # runs tsx lib/db/migrate && next build
```

### Deployment

**Admin website:** Google Cloud Run via `cloudbuild.yaml`
- Build args: `NEXT_PUBLIC_SUPABASE_URL`, `NEXT_PUBLIC_SUPABASE_ANON_KEY`
- Runtime env: `SUPABASE_SERVICE_ROLE_KEY`, Shopify keys
- See `admin-website/DEPLOY_CLOUD_RUN.md`

**Chatbot:** Vercel
- Env: `POSTGRES_URL`, `AUTH_SECRET`, `AI_GATEWAY_API_KEY`, `BLOB_READ_WRITE_TOKEN`
- Migrations auto-run on build via `package.json` build script

## Project-Specific Patterns

### Server Actions (Admin Website)

All mutations use server actions with this pattern:

```typescript
'use server'
import { revalidatePath } from 'next/cache'
import { createClient } from '@/lib/supabase/server'

export async function mutateData(formData: FormData) {
  // 1. Permission check (UX layer)
  // 2. Validation
  // 3. Database operation (RLS enforces security)
  // 4. Revalidate path
  return { success: boolean, error?: string }
}
```

Examples: `app/actions/auth.ts`, `app/actions/users.ts`, `app/actions/prescriptions.ts`

### Form Validation

React Hook Form + Zod + shadcn/ui components:

```typescript
const formSchema = z.object({
  email: z.string().email(),
  name: z.string().min(1),
})

type FormData = z.infer<typeof formSchema>

const form = useForm<FormData>({
  resolver: zodResolver(formSchema),
})
```

### Supabase Client Creation

```typescript
// Server components (admin website)
import { createClient } from '@/lib/supabase/server'
const supabase = await createClient() // uses cookies

// Client components (rarely needed)
import { createBrowserClient } from '@supabase/ssr'
const supabase = createBrowserClient(url, anonKey)
```

### Medical Timeline Pattern (Admin Website)

Parallel data fetching for patient details (see `app/doctor/patients/[id]/page.tsx`):

```typescript
const [patient, consultations, medications, questionnaires, treatments] = 
  await Promise.all([
    fetchPatient(id),
    fetchConsultations(id),
    fetchMedications(id),
    fetchQuestionnaires(id),
    fetchTreatments(id),
  ])
```

Aggregated into timeline component with chronological sorting.

### Drizzle Queries (Chatbot)

```typescript
import { db } from '@/lib/db'
import { chat, message } from '@/lib/db/schema'
import { eq } from 'drizzle-orm'

const chats = await db.select().from(chat).where(eq(chat.userId, userId))
```

## Key Files to Reference

### Type Safety
- `.cursor/rules/typescript-type-safety.mdc` - Type safety enforcement rules
- `admin-website/types/database.ts` - Generated Supabase types
- `andyou-chatbot/lib/db/schema.ts` - Drizzle schema definitions

### Authentication & RBAC
- `admin-website/docs/RBAC_IMPLEMENTATION.md` - Complete RBAC guide
- `admin-website/lib/rbac.ts` - Role helper functions
- `database/schema.sql` - RLS policies (lines 515+)

### Database Architecture
- `database/README.md` - Schema overview and migration history
- `database/ai-docs/backend-architecture.md` - Agentic workflow design

### Deployment
- `admin-website/cloudbuild.yaml` - GCP Cloud Run config
- `admin-website/Dockerfile` - Multi-stage build
- `andyou-chatbot/.env.example` - Required environment variables

## Common Pitfalls

1. **Type drift** - Always regenerate types after schema changes
2. **RLS bypass** - Never use service role except in `lib/supabase/admin.ts` with explicit permission checks
3. **Client-side security** - Role checks in UI are for UX only, RLS is the actual boundary
4. **Null handling** - Supabase queries can return `null`, always check before use
5. **Migration order** - Drizzle migrations go to `database/migrations/`, not `andyou-chatbot/migrations/`

## Commit Message Format

Use conventional commits (see `.cursor/rules/git-commits.mdc`):

```
feat(auth): add doctor role-based routing
fix(api): handle null patient assignments
docs(readme): update deployment steps
refactor(db): extract timeline query logic
```

Types: `feat | fix | docs | style | refactor | perf | test | build | ci | chore | revert`

## Testing

- **Admin website:** Manual testing + type checking (`tsc --noEmit`)
- **Chatbot:** Playwright tests (`pnpm test`)

Check errors before committing: `pnpm lint` (chatbot) or `npm run lint` (admin)
