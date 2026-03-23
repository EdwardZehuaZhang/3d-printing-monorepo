import type { Metadata } from "next";
import { IBM_Plex_Mono } from "next/font/google";
import Navigation from "@/components/Navigation";
import Footer from "@/components/Footer";
import "./globals.css";

const ibmPlexMono = IBM_Plex_Mono({
  subsets: ["latin"],
  weight: ["300", "400", "500", "600", "700"],
  variable: "--font-mono",
});

export const metadata: Metadata = {
  title: "SenseKit — Turn Any 3D Print Into a Touch Sensor",
  description:
    "The complete kit for making interactive, touch-sensitive 3D prints. Conductive filament, Rhino plugin, and SenseBoard included.",
  icons: {
    icon: "/SenseKit_icon.png",
    apple: "/SenseKit_icon.png",
  },
  openGraph: {
    images: ["/SenseKit_icon.png"],
  },
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en" className={ibmPlexMono.variable}>
      <body className="font-mono">
        <Navigation />
        <main>{children}</main>
        <Footer />
      </body>
    </html>
  );
}
