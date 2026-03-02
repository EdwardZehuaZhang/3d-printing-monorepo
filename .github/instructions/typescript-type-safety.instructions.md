---
applyTo: '**'
---
---
description: Enforce TypeScript type safety and prevent type inconsistencies
globs: ['**/*.ts', '**/*.tsx']
alwaysApply: true
---

# TypeScript Type Safety & Database Sync

Prevent build failures and linting errors caused by type inconsistencies.

## Core Principles

- Never use `any` type
- Always sync database schema changes with TypeScript types
- Use Zod for runtime validation that matches TypeScript types
- Regenerate types after schema changes

## Type Consistency Checklist

### Before Making Schema Changes

1. **Document the change** in migration comments
2. **Update all related type files**:
   - Database schema (`database/schema.sql`)
   - Drizzle schema (`andyou-chatbot/lib/db/schema.ts`)
   - Supabase types (`andyou-cms/types/database.ts`)
3. **Regenerate types**:
   ```bash
   # For Supabase types
   npx supabase gen types typescript --project-id <project-id> > types/database.ts
   
   # For Drizzle
   pnpm --filter andyou-chatbot db:generate
   ```

### Type Safety Rules

#### 1. Database Types
```typescript
// ✅ Good: Use generated types
import type { Database } from '@/types/database'
type Patient = Database['public']['Tables']['patients']['Row']

// ❌ Bad: Manual type definitions that can drift
type Patient = {
  id: string
  name: string
  // ... manually maintained
}
```

#### 2. API Responses
```typescript
// ✅ Good: Type API responses with database types
async function getPatient(id: string): Promise<Patient | null> {
  const { data } = await supabase
    .from('patients')
    .select('*')
    .eq('id', id)
    .single()
  return data
}

// ❌ Bad: Untyped or loosely typed
async function getPatient(id: string) {
  const { data } = await supabase
    .from('patients')
    .select('*')
    .eq('id', id)
    .single()
  return data // type is 'any' or inferred incorrectly
}
```

#### 3. Zod Schemas
```typescript
// ✅ Good: Zod schema matches TypeScript type
import { z } from 'zod'
import type { Database } from '@/types/database'

type PatientInsert = Database['public']['Tables']['patients']['Insert']

const patientSchema = z.object({
  name: z.string().min(1),
  email: z.string().email(),
  phone: z.string().optional(),
}) satisfies z.ZodType<Omit<PatientInsert, 'id' | 'created_at'>>

// ❌ Bad: Zod schema drifts from database type
const patientSchema = z.object({
  name: z.string(),
  email: z.string(),
  // Missing fields or wrong types
})
```

#### 4. Form Data
```typescript
// ✅ Good: Extract type from Zod schema
const formSchema = z.object({
  name: z.string().min(1),
  email: z.string().email(),
})

type FormData = z.infer<typeof formSchema>

// ❌ Bad: Separate type definition
type FormData = {
  name: string
  email: string
}
```

#### 5. Nullable Values
```typescript
// ✅ Good: Handle null explicitly
const patient: Patient | null = await getPatient(id)
if (!patient) {
  return { error: 'Patient not found' }
}
// Now patient is non-null

// ❌ Bad: Assume non-null
const patient = await getPatient(id)
patient.name // Could be null
```

#### 6. Strict Null Checks
```typescript
// ✅ Good: Check before use
if (user?.email) {
  await sendEmail(user.email)
}

// ❌ Bad: Non-null assertion without verification
await sendEmail(user!.email)
```

## Common Patterns

### Server Actions
```typescript
'use server'

import { z } from 'zod'
import type { Database } from '@/types/database'

const schema = z.object({
  patientId: z.string().uuid(),
  notes: z.string().min(1),
})

export async function updatePatientNotes(
  formData: FormData
): Promise<{ success: boolean; error?: string }> {
  const result = schema.safeParse({
    patientId: formData.get('patientId'),
    notes: formData.get('notes'),
  })

  if (!result.success) {
    return { success: false, error: result.error.message }
  }

  const { patientId, notes } = result.data
  
  // Type-safe database operation
  const { error } = await supabase
    .from('patients')
    .update({ notes })
    .eq('id', patientId)

  if (error) {
    return { success: false, error: error.message }
  }

  return { success: true }
}
```

### React Hook Form with Zod
```typescript
import { useForm } from 'react-hook-form'
import { zodResolver } from '@hookform/resolvers/zod'

const formSchema = z.object({
  name: z.string().min(1),
  email: z.string().email(),
})

type FormData = z.infer<typeof formSchema>

function PatientForm() {
  const form = useForm<FormData>({
    resolver: zodResolver(formSchema),
    defaultValues: {
      name: '',
      email: '',
    },
  })

  const onSubmit = (data: FormData) => {
    // data is fully typed
  }
}
```

## Type Generation Commands

Add to your workflow:

```bash
# After schema changes in andyou-cms (Supabase)
cd andyou-cms
npx supabase gen types typescript --local > types/database.ts

# After schema changes in andyou-chatbot (Drizzle)
cd andyou-chatbot
pnpm db:generate
pnpm db:migrate

# Check types before commit
tsc --noEmit
```

## Pre-commit Checklist

- [ ] Run `tsc --noEmit` to check for type errors
- [ ] Regenerate types if schema changed
- [ ] Update Zod schemas to match database types
- [ ] Run linter: `pnpm lint`
- [ ] Run build locally: `pnpm build`

## Common Type Errors to Avoid

1. **Assuming non-null**: Always check for null/undefined
2. **Using `any`**: Use `unknown` and type guards instead
3. **Skipping validation**: Always validate external data with Zod
4. **Stale types**: Regenerate after schema changes
5. **Type assertions**: Avoid `as` unless absolutely necessary
6. **Ignoring errors**: Never use `@ts-ignore` without a comment explaining why
